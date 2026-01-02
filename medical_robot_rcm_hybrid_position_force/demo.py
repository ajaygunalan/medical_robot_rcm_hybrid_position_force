"""
RCM Hybrid Controller Demos
===========================
Usage:
    python demo.py                              # RCM pivoting (no tissue)
    python demo.py --tissue vaginal             # RCM pivoting + vaginal tissue
    python demo.py --mode rcm_and_force         # RCM + force control (no tissue)
    python demo.py --mode rcm_and_force --tissue flat
    python demo.py --check                      # Position trocar interactively
    python demo.py --check --tissue flat        # Position trocar + tissue
"""
import sys
import argparse
import numpy as np
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from pydrake.all import (
    DiagramBuilder, AddMultibodyPlantSceneGraph, Parser,
    MeshcatVisualizer, StartMeshcat,
    RigidTransform, Sphere, Rgba,
    LeafSystem, BasicVector, AbstractValue, ContactResults,
    JacobianWrtVariable, Simulator, RollPitchYaw,
    Integrator, Multiplexer, InverseDynamicsController,
    Role, MeshcatVisualizerParams
)
from pydrake.geometry import (
    Mesh,
    ProximityProperties,
    AddCompliantHydroelasticProperties,
    AddRigidHydroelasticProperties,
    AddContactMaterial
)
from pydrake.multibody.plant import CoulombFriction, ContactModel
from controller import rcm_step, solve_rcm_qp, compute_rcm_error, compute_jacobians
from driver.ur5e_driver import UR5E_PACKAGE_XML

# --- CONFIGURATION ---
FORCE_TARGET = 5.0      # Newtons
K_FORCE = 0.005         # Admittance gain (m/s per N)
DT = 0.001

POSES = {
    "vaginal": {"p": [0.4919, 0.1723, 0.3458], "rpy": [-2.5160, 1.5708, -0.9452]},
    "flat": {"offset_z": -0.308}
}


# =============================================================================
# SHARED UTILITIES
# =============================================================================

def get_pivoting_twist(t, trocar_pos, tcp_pos, include_roll=True):
    """Sinusoidal pivoting motion around trocar."""
    rate = np.deg2rad(15) * 2 * np.pi * 0.5 * np.cos(2 * np.pi * 0.5 * t)

    if include_roll:
        cycle_time = t % 10.0
        if cycle_time < 5.0:
            omega = np.array([rate, 0, 0])  # Yaw
        else:
            omega = np.array([0, 0, rate])  # Roll
    else:
        cycle_time = t % 15.0
        if cycle_time < 5.0:
            omega = np.array([rate, 0, 0])  # Yaw
        elif cycle_time < 10.0:
            omega = np.array([0, 0, rate])  # Roll
        else:
            omega = np.array([0, rate, 0])  # Pitch

    r = tcp_pos - trocar_pos
    v = np.cross(omega, r)
    return np.concatenate([omega, v])


# =============================================================================
# RCM MODE (from rcm_demo.py)
# =============================================================================

def run_rcm(tissue_type=None):
    """Pure RCM pivoting demo - optional tissue visualization, no force control."""
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
    parser = Parser(plant)
    parser.package_map().AddPackageXml(UR5E_PACKAGE_XML)
    parser.AddModelsFromUrl("package://ur5e_description/ur5e_netft_probe.dmd.yaml")
    plant.Finalize()

    meshcat = StartMeshcat()
    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
    diagram = builder.Build()

    ctx = diagram.CreateDefaultContext()
    plant_ctx = plant.GetMyContextFromRoot(ctx)
    q = plant.GetPositions(plant_ctx)

    # Trocar position
    p_trocar_in_probe = np.array([0.0, 0.005, 0.06])
    probe_link = plant.GetFrameByName("transvaginal_probe_link")
    tcp_frame = plant.GetFrameByName("tcp")
    world = plant.world_frame()
    X_W_Probe = plant.CalcRelativeTransform(plant_ctx, world, probe_link)
    trocar_pos_world = X_W_Probe.multiply(p_trocar_in_probe)

    meshcat.SetObject("trocar", Sphere(0.015), Rgba(1, 0, 0, 1))
    meshcat.SetTransform("trocar", RigidTransform(trocar_pos_world))

    # Add tissue visualization if specified
    if tissue_type:
        X_W_TCP = plant.CalcRelativeTransform(plant_ctx, world, tcp_frame)
        if tissue_type == "flat":
            tissue_obj_path = str(Path(__file__).parent.parent / "models" / "tissue" / "flat" / "flat_tissue.obj")
            X_TCP_Tissue = RigidTransform(RollPitchYaw(0, 0, 0), [0, 0, POSES["flat"]["offset_z"]])
            tissue_pose = X_W_TCP.multiply(X_TCP_Tissue)
        else:
            tissue_obj_path = str(Path(__file__).parent.parent / "models" / "tissue" / "vaginal" / "vagina_tissue.obj")
            tissue_pose = RigidTransform(RollPitchYaw(POSES["vaginal"]["rpy"]), POSES["vaginal"]["p"])
        meshcat.SetObject("tissue", Mesh(tissue_obj_path, scale=1.0), Rgba(0.1, 0.2, 0.6, 0.7))  # Dark blue
        meshcat.SetTransform("tissue", tissue_pose)

    diagram.ForcedPublish(ctx)

    tissue_str = tissue_type if tissue_type else "no tissue"
    print(f"--- RCM Pivoting Demo ({tissue_str}) ---")
    print(f"Trocar: {p_trocar_in_probe}")
    print(f"Meshcat: {meshcat.web_url()}")
    input("Press Enter to start...")

    t = 0.0
    try:
        while True:
            tcp = plant.GetFrameByName("tcp")
            plant.SetPositions(plant_ctx, q)
            X_W_TCP = plant.CalcRelativeTransform(plant_ctx, world, tcp)

            V_des = get_pivoting_twist(t, trocar_pos_world, X_W_TCP.translation(), include_roll=False)
            v_opt = rcm_step(plant, plant_ctx, q, trocar_pos_world, V_des,
                             shaft_frame="transvaginal_probe_link")

            q = q + v_opt * 0.01
            plant.SetPositions(plant_ctx, q)
            diagram.ForcedPublish(ctx)
            t += 0.01
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\nStopped.")


# =============================================================================
# HYBRID FORCE MODE (from rcm_force_control.py)
# =============================================================================

class RcmHybridController(LeafSystem):
    def __init__(self, plant, trocar_pos_world, probe_body_name="transvaginal_probe_link"):
        LeafSystem.__init__(self)
        self.plant = plant
        self.context = plant.CreateDefaultContext()
        self.trocar_pos = trocar_pos_world

        self.probe_body = plant.GetBodyByName(probe_body_name)
        self.probe_frame = self.probe_body.body_frame()
        self.tcp_frame = plant.GetFrameByName("tcp")
        self.world = plant.world_frame()
        self.ur5e_model = plant.GetModelInstanceByName("ur5e")

        self.DeclareVectorInputPort("robot_state", plant.num_multibody_states(self.ur5e_model))
        self.DeclareAbstractInputPort("contact_results", AbstractValue.Make(ContactResults()))
        self.DeclareVectorOutputPort("joint_velocities", plant.num_velocities(self.ur5e_model), self.calc_output)

    def calc_output(self, context, output):
        t = context.get_time()
        state = self.get_input_port(0).Eval(context)
        self.plant.SetPositions(self.context, self.ur5e_model, state[:6])

        X_WP = self.plant.CalcRelativeTransform(self.context, self.world, self.probe_frame)
        X_WT = self.plant.CalcRelativeTransform(self.context, self.world, self.tcp_frame)

        # Force sensing
        contact_results = self.get_input_port(1).Eval(context)
        total_force_world = np.zeros(3)
        for i in range(contact_results.num_point_pair_contacts()):
            info = contact_results.point_pair_contact_info(i)
            if info.bodyB_index() == self.probe_body.index():
                total_force_world += info.contact_force()
            elif info.bodyA_index() == self.probe_body.index():
                total_force_world -= info.contact_force()

        n_hat = X_WP.rotation().matrix()[:, 2]
        f_compression = -np.dot(total_force_world, n_hat)

        # Scanning motion (tangent)
        V_pos_world = get_pivoting_twist(t, self.trocar_pos, X_WT.translation())
        v_linear_scan = V_pos_world[3:]
        v_tangent = v_linear_scan - np.dot(v_linear_scan, n_hat) * n_hat

        # Admittance control (normal)
        v_force_mag = K_FORCE * (FORCE_TARGET - f_compression)
        v_force_mag = np.clip(v_force_mag, -0.05, 0.05)
        v_force_vector = v_force_mag * n_hat

        v_linear_cmd = v_tangent + v_force_vector
        V_des = np.concatenate([V_pos_world[:3], v_linear_cmd])

        # Prioritized weighting
        w_force = 100.0
        W_lin = np.eye(3) + (w_force - 1.0) * np.outer(n_hat, n_hat)

        e, n_hat_calc, lam = compute_rcm_error(X_WP.translation(), X_WP.rotation().matrix(), self.trocar_pos)
        J_P, J_T = compute_jacobians(self.plant, self.context, self.probe_frame, self.tcp_frame, self.world, n_hat_calc, lam)
        v_opt = solve_rcm_qp(J_P, J_T, e, n_hat_calc, V_des, W_lin=W_lin)
        output.SetFromVector(v_opt)


class ForceVisualizer(LeafSystem):
    def __init__(self, meshcat, plant, probe_body_name="transvaginal_probe_link"):
        LeafSystem.__init__(self)
        self.meshcat = meshcat
        self.probe_body = plant.GetBodyByName(probe_body_name)
        self.tcp_frame = plant.GetFrameByName("tcp")
        self.world = plant.world_frame()
        self.plant = plant
        self.DeclareAbstractInputPort("contact_results", AbstractValue.Make(ContactResults()))
        self.DeclareVectorInputPort("robot_state", plant.num_multibody_states(plant.GetModelInstanceByName("ur5e")))
        self.DeclarePeriodicPublishEvent(0.05, 0.0, self.PublishForce)

    def PublishForce(self, context):
        state = self.get_input_port(1).Eval(context)
        plant_context = self.plant.CreateDefaultContext()
        self.plant.SetPositions(plant_context, self.plant.GetModelInstanceByName("ur5e"), state[:6])
        X_WT = self.plant.CalcRelativeTransform(plant_context, self.world, self.tcp_frame)
        p_tcp = X_WT.translation()

        contact_results = self.get_input_port(0).Eval(context)
        total_force = np.zeros(3)
        for i in range(contact_results.num_point_pair_contacts()):
            info = contact_results.point_pair_contact_info(i)
            if info.bodyB_index() == self.probe_body.index():
                total_force += info.contact_force()
            elif info.bodyA_index() == self.probe_body.index():
                total_force -= info.contact_force()

        f_norm = np.linalg.norm(total_force)
        if f_norm > 0.1:
            p_end = p_tcp + total_force * 0.02
            color = Rgba(0, 1, 0, 1) if f_norm < FORCE_TARGET else Rgba(1, 0, 0, 1)
            self.meshcat.SetLine("force_vector", np.vstack([p_tcp, p_end]).T, 5.0, color)
        else:
            self.meshcat.SetLine("force_vector", np.zeros((3, 2)), 1.0, Rgba(0, 0, 0, 0))


def run_hybrid_force(tissue_type=None):
    """RCM + hybrid force/position control. tissue_type=None means no tissue (0N force)."""
    # First pass: get TCP pose for tissue placement (if needed)
    tissue_pose = None
    tissue_obj_path = None
    if tissue_type:
        builder = DiagramBuilder()
        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=1e-3)
        parser = Parser(plant)
        parser.package_map().AddPackageXml(UR5E_PACKAGE_XML)
        parser.AddModelsFromUrl("package://ur5e_description/ur5e_netft_probe.dmd.yaml")
        plant.Finalize()

        temp_ctx = plant.CreateDefaultContext()
        tcp_frame = plant.GetFrameByName("tcp")
        X_W_TCP = plant.CalcRelativeTransform(temp_ctx, plant.world_frame(), tcp_frame)

        if tissue_type == "flat":
            tissue_obj_path = str(Path(__file__).parent.parent / "models" / "tissue" / "flat" / "flat_tissue.obj")
            X_TCP_Tissue = RigidTransform(RollPitchYaw(0, 0, 0), [0, 0, POSES["flat"]["offset_z"]])
            tissue_pose = X_W_TCP.multiply(X_TCP_Tissue)
        else:
            tissue_obj_path = str(Path(__file__).parent.parent / "models" / "tissue" / "vaginal" / "vagina_tissue.obj")
            tissue_pose = RigidTransform(RollPitchYaw(POSES["vaginal"]["rpy"]), POSES["vaginal"]["p"])

    # Main build (with or without tissue)
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=1e-3)
    plant.set_contact_model(ContactModel.kHydroelasticWithFallback)
    parser = Parser(plant)
    parser.package_map().AddPackageXml(UR5E_PACKAGE_XML)
    parser.AddModelsFromUrl("package://ur5e_description/ur5e_netft_probe.dmd.yaml")

    if tissue_type:
        # Visual geometry - original OBJ
        plant.RegisterVisualGeometry(plant.world_body(), tissue_pose, Mesh(tissue_obj_path, scale=1.0), "tissue_vis", [0.1, 0.2, 0.6, 1.0])  # Dark blue

        # Collision geometry - RIGID hydroelastic preserves OBJ surface exactly
        # Probe is compliant, so rigid tissue + compliant probe = contact works
        tissue_props = ProximityProperties()
        AddRigidHydroelasticProperties(properties=tissue_props)
        AddContactMaterial(
            properties=tissue_props,
            dissipation=1.0,
            friction=CoulombFriction(1.0, 1.0)
        )
        plant.RegisterCollisionGeometry(plant.world_body(), tissue_pose, Mesh(tissue_obj_path, scale=1.0), "tissue_col", tissue_props)

    plant.Finalize()

    meshcat = StartMeshcat()
    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat, MeshcatVisualizerParams(role=Role.kProximity, prefix="collision", visible_by_default=False))

    ctx = plant.CreateDefaultContext()
    probe_link = plant.GetFrameByName("transvaginal_probe_link")
    q0 = plant.GetPositions(ctx)
    X_WP_0 = plant.CalcRelativeTransform(ctx, plant.world_frame(), probe_link)
    p_trocar_in_probe = np.array([0.0, 0.005, 0.06])
    trocar_pos_world = X_WP_0.multiply(p_trocar_in_probe)

    meshcat.SetObject("trocar", Sphere(0.015), Rgba(1, 0, 0, 1))
    meshcat.SetTransform("trocar", RigidTransform(trocar_pos_world))

    force_vis = builder.AddSystem(ForceVisualizer(meshcat, plant))
    builder.Connect(plant.get_contact_results_output_port(), force_vis.get_input_port(0))
    builder.Connect(plant.get_state_output_port(), force_vis.get_input_port(1))

    controller = builder.AddSystem(RcmHybridController(plant, trocar_pos_world))
    builder.Connect(plant.get_state_output_port(), controller.get_input_port(0))
    builder.Connect(plant.get_contact_results_output_port(), controller.get_input_port(1))

    integ = builder.AddSystem(Integrator(6))
    inv_dyn = builder.AddSystem(InverseDynamicsController(plant, [2000]*6, [100]*6, [100]*6, False))
    builder.Connect(controller.get_output_port(0), integ.get_input_port())
    mux = builder.AddSystem(Multiplexer([6, 6]))
    builder.Connect(integ.get_output_port(), mux.get_input_port(0))
    builder.Connect(controller.get_output_port(0), mux.get_input_port(1))
    builder.Connect(mux.get_output_port(), inv_dyn.get_input_port_desired_state())
    builder.Connect(plant.get_state_output_port(), inv_dyn.get_input_port_estimated_state())
    builder.Connect(inv_dyn.get_output_port(0), plant.get_actuation_input_port())

    diagram = builder.Build()
    sim = Simulator(diagram)
    sim.set_target_realtime_rate(1.0)
    sim_ctx = sim.get_mutable_context()
    integ.set_integral_value(integ.GetMyContextFromRoot(sim_ctx), q0)
    plant.SetPositions(plant.GetMyContextFromRoot(sim_ctx), q0)

    diagram.ForcedPublish(sim_ctx)
    tissue_str = tissue_type if tissue_type else "no tissue"
    print(f"--- RCM + Force Control ({tissue_str}) ---")
    print(f"Target Force: {FORCE_TARGET}N")
    print(f"Meshcat: {meshcat.web_url()}")
    input("Press Enter to start simulation...")

    try:
        while True:
            sim.AdvanceTo(sim_ctx.get_time() + 0.1)
    except KeyboardInterrupt:
        pass
    print("Done.")


# =============================================================================
# CHECK MODE (from check_trocar.py and check_trocar_tissue.py)
# =============================================================================

def run_check(tissue_type=None):
    """Interactive slider utility for positioning trocar and optionally tissue."""
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
    parser = Parser(plant)
    parser.package_map().AddPackageXml(UR5E_PACKAGE_XML)
    parser.AddModelsFromUrl("package://ur5e_description/ur5e_netft_probe.dmd.yaml")
    plant.Finalize()

    meshcat = StartMeshcat()
    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
    diagram = builder.Build()

    context = diagram.CreateDefaultContext()
    plant_context = plant.GetMyContextFromRoot(context)

    probe_link = plant.GetFrameByName("transvaginal_probe_link")
    tcp_frame = plant.GetFrameByName("tcp")
    world = plant.world_frame()

    # Trocar sliders
    meshcat.AddSlider("trocar_x", -0.1, 0.1, 0.001, 0.0)
    meshcat.AddSlider("trocar_y", -0.1, 0.1, 0.001, 0.005)
    meshcat.AddSlider("trocar_z", 0.0, 0.4, 0.001, 0.06)
    meshcat.SetObject("trocar", Sphere(0.015), Rgba(1, 0, 0, 1))

    # Tissue sliders (if enabled)
    if tissue_type:
        meshcat.AddSlider("tissue_x", -0.5, 0.5, 0.001, 0.0)
        meshcat.AddSlider("tissue_y", -0.5, 0.5, 0.001, 0.0)
        meshcat.AddSlider("tissue_z", -0.5, 0.5, 0.001, 0.0)
        meshcat.AddSlider("tissue_roll", -3.14, 3.14, 0.01, 0.0)
        meshcat.AddSlider("tissue_pitch", -3.14, 3.14, 0.01, 0.0)
        meshcat.AddSlider("tissue_yaw", -3.14, 3.14, 0.01, 0.0)

        if tissue_type == "flat":
            tissue_obj_path = str(Path(__file__).parent.parent / "models" / "tissue" / "flat" / "flat_tissue.obj")
        else:
            tissue_obj_path = str(Path(__file__).parent.parent / "models" / "tissue" / "vaginal" / "vagina_tissue.obj")
        meshcat.SetObject("tissue", Mesh(tissue_obj_path, scale=1.0), Rgba(0.1, 0.2, 0.6, 0.7))  # Dark blue

    diagram.ForcedPublish(context)

    print(f"Meshcat: {meshcat.web_url()}")
    print("Controls:")
    print("  trocar_*: Position red dot relative to PROBE LINK")
    if tissue_type:
        print("  tissue_*: Position tissue mesh relative to TCP")
    print("Press Ctrl+C to exit and get coordinates.")

    tx = ty = tz = 0.0
    tis_x = tis_y = tis_z = tis_r = tis_p = tis_yaw = 0.0

    try:
        while True:
            # Trocar
            tx = meshcat.GetSliderValue("trocar_x")
            ty = meshcat.GetSliderValue("trocar_y")
            tz = meshcat.GetSliderValue("trocar_z")

            X_W_Probe = plant.CalcRelativeTransform(plant_context, world, probe_link)
            p_trocar_world = X_W_Probe.multiply([tx, ty, tz])
            meshcat.SetTransform("trocar", RigidTransform(p_trocar_world))

            # Tissue
            if tissue_type:
                tis_x = meshcat.GetSliderValue("tissue_x")
                tis_y = meshcat.GetSliderValue("tissue_y")
                tis_z = meshcat.GetSliderValue("tissue_z")
                tis_r = meshcat.GetSliderValue("tissue_roll")
                tis_p = meshcat.GetSliderValue("tissue_pitch")
                tis_yaw = meshcat.GetSliderValue("tissue_yaw")

                X_TCP_Tissue = RigidTransform(RollPitchYaw(tis_r, tis_p, tis_yaw), [tis_x, tis_y, tis_z])
                X_W_TCP = plant.CalcRelativeTransform(plant_context, world, tcp_frame)
                X_W_Tissue = X_W_TCP.multiply(X_TCP_Tissue)
                meshcat.SetTransform("tissue", X_W_Tissue)

            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\n--- Final Configuration ---")
        print(f"Trocar (in Probe Frame): [{tx:.4f}, {ty:.4f}, {tz:.4f}]")
        if tissue_type:
            print(f"Tissue Pose (Relative to TCP):")
            print(f"  Translation: [{tis_x:.4f}, {tis_y:.4f}, {tis_z:.4f}]")
            print(f"  Rotation (RPY): [{tis_r:.4f}, {tis_p:.4f}, {tis_yaw:.4f}]")


# =============================================================================
# CLI
# =============================================================================

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="RCM Hybrid Controller Demos",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python demo.py                                    # RCM pivoting (no tissue)
  python demo.py --tissue vaginal                   # RCM pivoting + vaginal tissue
  python demo.py --mode rcm_and_force               # RCM + force (no tissue, 0N)
  python demo.py --mode rcm_and_force --tissue flat # RCM + force + flat tissue
  python demo.py --check                            # Position trocar only
  python demo.py --check --tissue flat              # Position trocar + tissue
        """
    )

    parser.add_argument("--mode", choices=["rcm", "rcm_and_force"], default="rcm",
                        help="Control mode (default: rcm)")
    parser.add_argument("--tissue", choices=["flat", "vaginal"], default=None,
                        help="Tissue type (optional). If not specified, no tissue is loaded")
    parser.add_argument("--check", action="store_true",
                        help="Run interactive position check utility")

    args = parser.parse_args()

    if args.check:
        run_check(args.tissue)
    elif args.mode == "rcm":
        run_rcm(args.tissue)
    elif args.mode == "rcm_and_force":
        run_hybrid_force(args.tissue)
