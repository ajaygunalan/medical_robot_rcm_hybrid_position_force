"""UR5e Driver - Digital Twin pattern for unified sim/hardware control.

The driver owns an internal Drake plant that:
- Always exists (sim and hardware modes)
- Provides plant/context for Jacobian computation
- Provides Meshcat visualization
- Syncs with real robot state in hardware mode

Usage:
    # Simulation
    driver = UR5eDriver(hardware=False)

    # Hardware
    driver = UR5eDriver(hardware=True, robot_ip="192.168.1.1")

    # Control loop (identical for both)
    plant, ctx = driver.get_plant_context()
    while running:
        q, qd = driver.get_state()
        v_cmd = compute_velocity(plant, ctx, q)  # Use plant for Jacobians
        driver.send_velocity(v_cmd)
        driver.step()
    driver.stop()
"""
import numpy as np
import time
from pathlib import Path
from typing import Tuple, Optional

from pydrake.all import (
    DiagramBuilder, AddMultibodyPlantSceneGraph, Parser,
    MeshcatVisualizer, StartMeshcat, MultibodyPlant, Context, Meshcat
)

UR5E_PACKAGE_XML = str(Path(__file__).parent.parent / 'models' / 'package.xml')

try:
    from ur_rtde import RTDEControlInterface, RTDEReceiveInterface
    _UR_RTDE_AVAILABLE = True
except ImportError:
    _UR_RTDE_AVAILABLE = False


class UR5eDriver:
    """Digital Twin driver for UR5e - unified interface for sim and hardware.

    The driver owns an internal Drake plant that mirrors the robot state.
    In simulation mode, state is integrated internally.
    In hardware mode, state is read from the real robot and synced to the plant.
    """

    def __init__(self, hardware: bool = False, robot_ip: Optional[str] = None,
                 meshcat: Optional[Meshcat] = None, finalize: bool = True,
                 dt: float = 0.002):
        """Create driver with internal Drake plant.

        Args:
            hardware: False=simulation, True=real robot
            robot_ip: Robot IP address (required if hardware=True)
            meshcat: Existing Meshcat instance, or None to create new one
            finalize: If True, finalize plant immediately.
                     If False, caller must call driver.finalize() after
                     adding extra geometry (tissue, trocar, etc.)
            dt: Control loop period (default 0.002 = 500Hz)
        """
        self._hardware = hardware
        self._dt = dt
        self._finalized = False
        self._last_viz_time = 0.0
        self._viz_period = 1.0 / 30.0  # 30Hz visualization

        # Hardware connections
        self._rtde_c = None
        self._rtde_r = None
        if hardware:
            if not _UR_RTDE_AVAILABLE:
                raise ImportError("ur_rtde not installed. Run: pip install ur-rtde")
            if robot_ip is None:
                raise ValueError("robot_ip required for hardware mode")
            self._rtde_c = RTDEControlInterface(robot_ip)
            self._rtde_r = RTDEReceiveInterface(robot_ip)
            self._rtde_c.initPeriod()

        # Internal Drake plant (Digital Twin)
        self._builder = DiagramBuilder()
        self._plant, self._scene_graph = AddMultibodyPlantSceneGraph(
            self._builder, time_step=0.0
        )

        parser = Parser(self._plant)
        parser.package_map().AddPackageXml(UR5E_PACKAGE_XML)
        parser.AddModelsFromUrl("package://ur5e_description/ur5e_netft_probe.dmd.yaml")

        # Meshcat visualization
        self._meshcat = meshcat if meshcat else StartMeshcat()

        # State tracking
        self._q = np.array([0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0])  # equilibrium
        self._qd = np.zeros(6)
        self._v_cmd = np.zeros(6)

        if finalize:
            self.finalize()

    def finalize(self):
        """Finalize the plant after adding extra geometry.

        Must be called before control loop if finalize=False was used in __init__.
        """
        if self._finalized:
            return

        self._plant.Finalize()
        self._visualizer = MeshcatVisualizer.AddToBuilder(
            self._builder, self._scene_graph, self._meshcat
        )
        self._diagram = self._builder.Build()
        self._context = self._diagram.CreateDefaultContext()
        self._plant_ctx = self._plant.GetMyContextFromRoot(self._context)
        self._vis_ctx = self._visualizer.GetMyContextFromRoot(self._context)

        # Sync initial state
        if self._hardware:
            self._q = np.array(self._rtde_r.getActualQ())
            self._qd = np.array(self._rtde_r.getActualQd())

        self._plant.SetPositions(self._plant_ctx, self._q)
        self._visualizer.ForcedPublish(self._vis_ctx)

        self._finalized = True

    def get_state(self) -> Tuple[np.ndarray, np.ndarray]:
        """Returns (q, qd) - joint positions and velocities.

        Also syncs internal plant to current state (Digital Twin).
        """
        self._ensure_finalized()

        if self._hardware:
            self._q = np.array(self._rtde_r.getActualQ())
            self._qd = np.array(self._rtde_r.getActualQd())

        # Sync plant to current state
        self._plant.SetPositions(self._plant_ctx, self._q)

        return self._q.copy(), self._qd.copy()

    def send_velocity(self, qd: np.ndarray):
        """Send joint velocity command."""
        self._ensure_finalized()
        self._v_cmd = np.array(qd)

        if self._hardware:
            self._rtde_c.speedJ(self._v_cmd.tolist(), acceleration=0.5, time=self._dt)

    def step(self, dt: Optional[float] = None):
        """Advance simulation or pace hardware loop.

        Simulation: integrate q += v * dt, update visualization
        Hardware: waitPeriod() for precise timing, update visualization

        Visualization is throttled to ~30Hz.
        """
        self._ensure_finalized()
        dt = dt if dt is not None else self._dt

        if self._hardware:
            self._rtde_c.waitPeriod()
        else:
            # Kinematic integration for simulation
            self._q = self._q + self._v_cmd * dt
            self._plant.SetPositions(self._plant_ctx, self._q)
            time.sleep(dt)

        # Throttled visualization
        now = time.time()
        if now - self._last_viz_time >= self._viz_period:
            self._visualizer.ForcedPublish(self._vis_ctx)
            self._last_viz_time = now

    def stop(self):
        """Stop robot motion and cleanup."""
        self._v_cmd = np.zeros(6)

        if self._hardware and self._rtde_c:
            self._rtde_c.speedStop(0.5)
            self._rtde_c.stopScript()

    def get_plant_context(self) -> Tuple[MultibodyPlant, Context]:
        """Returns (plant, context) for Jacobian computation.

        The plant is synced to current robot state via get_state().
        Use this for controller computations.
        """
        self._ensure_finalized()
        return self._plant, self._plant_ctx

    @property
    def plant(self) -> MultibodyPlant:
        """Direct access to plant for adding geometry (before finalize)."""
        return self._plant

    @property
    def scene_graph(self):
        """Direct access to scene_graph for adding geometry (before finalize)."""
        return self._scene_graph

    @property
    def meshcat(self) -> Meshcat:
        """Direct access to Meshcat for adding visualization."""
        return self._meshcat

    @property
    def is_hardware(self) -> bool:
        """True if running on real hardware."""
        return self._hardware

    def get_tcp_force(self) -> np.ndarray:
        """Get TCP force/torque (6D). Hardware only, returns zeros in sim."""
        if self._hardware:
            return np.array(self._rtde_r.getActualTCPForce())
        return np.zeros(6)

    def move_to(self, q_target: np.ndarray, speed: float = 0.5, blocking: bool = True):
        """Move to joint configuration (blocking moveJ in hardware, instant in sim).

        Args:
            q_target: Target joint positions
            speed: Maximum joint speed (rad/s)
            blocking: If True, wait until motion completes
        """
        self._ensure_finalized()

        if self._hardware:
            self._rtde_c.moveJ(list(q_target), speed, 0.5, not blocking)
            if blocking:
                while not np.allclose(self._rtde_r.getActualQ(), q_target, atol=0.01):
                    self._q = np.array(self._rtde_r.getActualQ())
                    self._plant.SetPositions(self._plant_ctx, self._q)
                    self._visualizer.ForcedPublish(self._vis_ctx)
                    time.sleep(0.05)
        else:
            # Instant move in simulation
            self._q = np.array(q_target)
            self._plant.SetPositions(self._plant_ctx, self._q)
            self._visualizer.ForcedPublish(self._vis_ctx)

    def _ensure_finalized(self):
        """Raise error if plant not finalized."""
        if not self._finalized:
            raise RuntimeError("Driver not finalized. Call driver.finalize() first.")
