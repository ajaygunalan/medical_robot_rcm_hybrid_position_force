"""
RCM QP Controller - Simple functions matching rcm_qp_control.md
"""
import numpy as np
from pydrake.all import MathematicalProgram, OsqpSolver, JacobianWrtVariable


def skew(v):
    """Skew-symmetric matrix [v]x (Section 7)"""
    return np.array([[0, -v[2], v[1]],
                     [v[2], 0, -v[0]],
                     [-v[1], v[0], 0]])


def lateral_basis(n_hat):
    """B_perp: 2x3 orthonormal basis perpendicular to shaft (Section 6)"""
    aux = np.array([1, 0, 0]) if abs(n_hat[0]) < 0.9 else np.array([0, 1, 0])
    u1 = np.cross(n_hat, aux)
    u1 /= np.linalg.norm(u1)
    u2 = np.cross(n_hat, u1)
    return np.vstack([u1, u2])


def compute_rcm_error(p_F, R_WF, p_C):
    """
    Projection point P and lateral error (Section 6, Steps 1-5)

    Returns: e (lateral error), n_hat (shaft direction), lam (projection distance)
    """
    n_hat = R_WF[:, 2]              # Shaft = flange Z-axis
    r = p_C - p_F                   # Flange to trocar
    lam = r @ n_hat                 # Projection distance along shaft
    p_P = p_F + lam * n_hat         # Closest point on shaft to trocar
    e = p_C - p_P                   # Lateral error
    return e, n_hat, lam


def compute_jacobians(plant, ctx, flange, tcp, world, n_hat, lam):
    """
    Compute J_P and J_T Jacobians (Section 7)
    """
    # Flange Jacobian
    J_F = plant.CalcJacobianSpatialVelocity(
        ctx, JacobianWrtVariable.kQDot, flange, [0, 0, 0], world, world
    )[:, :6]
    J_omega, J_v = J_F[:3], J_F[3:]

    # J_P = J_v - [r_FP]x @ J_omega (Section 7)
    J_P = J_v - skew(lam * n_hat) @ J_omega

    # TCP (tip) Jacobian
    J_T = plant.CalcJacobianSpatialVelocity(
        ctx, JacobianWrtVariable.kQDot, tcp, [0, 0, 0], world, world
    )[:, :6]

    return J_P, J_T


def solve_rcm_qp(J_P, J_T, e, n_hat, V_des,
                 K_rcm=20.0, W_lin=None, w_roll=0.1, eps=0.01, v_max=1.5):
    """
    Solve RCM QP (Sections 4-5)

    min  (J_lin v - v_des).T @ W_lin @ (J_lin v - v_des) + ...
    s.t. B_perp @ J_P @ v = K_rcm * B_perp @ e
         |v| <= v_max
    """
    prog = MathematicalProgram()
    v = prog.NewContinuousVariables(6, "v")

    # RCM constraint: 2 equations, full rank (Section 4)
    B = lateral_basis(n_hat)
    prog.AddLinearEqualityConstraint(B @ J_P, K_rcm * B @ e, v)

    # Build cost matrices explicitly (guaranteed PSD)
    J_lin = J_T[3:]  # 3x6
    J_ang = J_T[:3]  # 3x6
    
    # Linear Weighting (Hybrid support)
    if W_lin is None:
        W_lin = np.eye(3)

    # Position cost: (J_lin v - v_des).T W_lin (J_lin v - v_des)
    # Roll cost: w_roll * ||n_hat.T @ J_ang @ v - n_hat.T @ omega_des||²
    # Regularization: eps * ||v||²

    J_roll = (n_hat @ J_ang).reshape(1, 6)  # 1x6
    omega_roll_des = np.array([n_hat @ V_des[:3]])  # (1,)

    # H = J_lin.T @ W_lin @ J_lin + w_roll * J_roll.T @ J_roll + eps * I
    H = J_lin.T @ W_lin @ J_lin + w_roll * J_roll.T @ J_roll + eps * np.eye(6)

    # f = -J_lin.T @ W_lin @ v_des - w_roll * J_roll.T @ omega_roll_des
    f = -J_lin.T @ W_lin @ V_des[3:] - w_roll * J_roll.T @ omega_roll_des

    prog.AddQuadraticCost(H, f, v)
    prog.AddBoundingBoxConstraint(-v_max, v_max, v)

    result = OsqpSolver().Solve(prog)
    if not result.is_success():
        print(f"QP Failed! Result: {result.get_solver_id().name()} status: {result.get_solution_result()}")
        return np.zeros(6)
    return result.GetSolution(v)


def rcm_step(plant, ctx, q, p_C, V_des, model_name="ur5e", tcp_frame="tcp", shaft_frame="flange", **kwargs):
    """
    Single control step: q -> v_opt

    Args:
        plant: Drake MultibodyPlant
        ctx: Plant context
        q: Joint positions (6,)
        p_C: Trocar position in world (3,)
        V_des: Desired tip twist [omega; v] (6,)
        tcp_frame: Name of TCP frame (default "tcp")
        shaft_frame: Name of the frame defining the shaft axis (Z-axis)
        **kwargs: Passed to solve_rcm_qp (e.g., W_lin)

    Returns:
        v_opt: Optimal joint velocities (6,)
    """
    model = plant.GetModelInstanceByName(model_name)
    world = plant.world_frame()
    shaft = plant.GetFrameByName(shaft_frame)
    tcp = plant.GetFrameByName(tcp_frame)

    plant.SetPositions(ctx, model, q)
    X = plant.CalcRelativeTransform(ctx, world, shaft)

    # Error and Jacobians relative to the SHAFT frame
    e, n_hat, lam = compute_rcm_error(X.translation(), X.rotation().matrix(), p_C)
    J_P, J_T = compute_jacobians(plant, ctx, shaft, tcp, world, n_hat, lam)

    return solve_rcm_qp(J_P, J_T, e, n_hat, V_des, **kwargs)
