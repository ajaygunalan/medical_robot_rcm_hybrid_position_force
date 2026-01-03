#!/usr/bin/env python3
"""UR5e driver test script with interactive multi-target control.

Same code runs in simulation and hardware - uses UR5eDriver Digital Twin.

Interactive Mode:
- Robot starts at equilibrium position
- Press [a]/[b]/[h]/[e] to move to target pose
- Press [s] during motion to stop
- Press [Esc] to exit

Usage:
    # Simulation
    python driver/example.py --sim

    # Hardware
    python driver/example.py --hardware --ip 169.254.120.1
"""
import argparse
import numpy as np
import sys
import select
import termios
import tty
import time
from contextlib import contextmanager

from pydrake.trajectories import PiecewisePolynomial

from ur5e_driver import UR5eDriver

# =============================================================================
# CONFIGURATION
# =============================================================================

POSES = {
    "home": [0.0, -1.5708, 0.0, -1.5708, -1.5708, 0.0],
    "equilibrium": [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0],
    "pose_a": [0.043128, -1.28824, 1.37179, -1.82208, -1.63632, -0.18],
    "pose_b": [0.30493, -0.982258, 0.955637, -1.48215, -1.72737, 0.204445],
}

KEY_MAP = {'a': 'pose_a', 'b': 'pose_b', 'h': 'home', 'e': 'equilibrium'}
KP_TRACKING = 0.5   # Small feedback gain for trajectory tracking
MAX_VELOCITY = 0.1  # rad/s - max joint velocity
MIN_DURATION = 0.5  # seconds - minimum trajectory duration
DT = 0.002          # 500 Hz control loop


# =============================================================================
# TRAJECTORY GENERATION
# =============================================================================

def make_trajectory(q_start: np.ndarray, q_target: np.ndarray) -> tuple:
    """Generate smooth trajectory from q_start to q_target.

    Uses Cubic polynomial with zero velocity at start/end for smooth motion.

    Returns:
        (trajectory, duration): PiecewisePolynomial and total duration in seconds
    """
    distance = np.max(np.abs(q_target - q_start))
    duration = max(distance / MAX_VELOCITY, MIN_DURATION)

    times = np.array([0.0, duration])
    positions = np.column_stack([q_start, q_target])
    zero_vel = np.zeros((6, 1))

    # Cubic with zero velocity at start and end (rest-to-rest motion)
    traj = PiecewisePolynomial.CubicWithContinuousSecondDerivatives(
        times, positions, zero_vel, zero_vel
    )
    return traj, duration


# =============================================================================
# TERMINAL UTILITIES
# =============================================================================

@contextmanager
def cbreak_mode():
    """Context manager for immediate key detection. Ctrl-C still works."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        yield
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def get_key(timeout=None):
    """Get key press. Returns None if timeout expires without input."""
    if timeout is None:
        return sys.stdin.read(1)
    if select.select([sys.stdin], [], [], timeout)[0]:
        return sys.stdin.read(1)
    return None


# =============================================================================
# CONTROL LOOP
# =============================================================================

def run_interactive(driver: UR5eDriver):
    """Unified interactive control loop for sim and hardware.

    Uses trajectory-based control with feedforward + feedback:
        v_cmd = v_trajectory + Kp * (q_trajectory - q_actual)
    """
    current_name = "start"
    trajectory = None
    traj_start_time = 0.0
    duration = 0.0

    mode_name = "HARDWARE" if driver.is_hardware else "SIMULATION"
    print(f"\n=== {mode_name} (Trajectory Control) ===")
    print(f"Meshcat: {driver.meshcat.web_url()}")
    print("\nControls: [a] pose_a  [b] pose_b  [h] home  [e] equilibrium")
    print("          [s] stop    [Esc] exit\n")

    with cbreak_mode():
        while True:
            q_actual, _ = driver.get_state()

            # Trajectory-based control with feedforward + feedback
            if trajectory is not None:
                t = time.time() - traj_start_time

                if t >= duration:
                    # Trajectory complete
                    driver.send_velocity(np.zeros(6))
                    print(f"Reached {current_name}")
                    trajectory = None
                else:
                    # Sample trajectory at current time
                    q_des = trajectory.value(t).flatten()
                    v_des = trajectory.derivative(1).value(t).flatten()

                    # Feedforward + feedback control
                    v_cmd = v_des + KP_TRACKING * (q_des - q_actual)
                    driver.send_velocity(v_cmd)
            else:
                driver.send_velocity(np.zeros(6))

            driver.step(dt=DT)

            # Check for key press (non-blocking)
            key = get_key(timeout=0)
            if key is None:
                continue

            if key == '\x1b':  # Escape
                print("\nExiting...")
                break

            if key == 's':
                trajectory = None
                driver.send_velocity(np.zeros(6))
                print("Stopped")
                continue

            if key in KEY_MAP:
                target_name = KEY_MAP[key]
                q_target = np.array(POSES[target_name])

                if np.allclose(q_actual, q_target, atol=0.01):
                    print(f"Already at {target_name}")
                    continue

                # Generate smooth trajectory
                trajectory, duration = make_trajectory(q_actual, q_target)
                traj_start_time = time.time()
                current_name = target_name
                print(f"Moving to {target_name}... ({duration:.1f}s, 's' to stop)")


# =============================================================================
# MAIN
# =============================================================================

def main():
    parser = argparse.ArgumentParser(
        description="UR5e interactive control (unified sim/hardware)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Controls:
  [a] pose_a   [b] pose_b   [h] home   [e] equilibrium
  [s] stop     [Esc] exit

Examples:
  python driver/example.py --sim
  python driver/example.py --hardware --ip 169.254.120.1
"""
    )

    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--sim", action="store_true", help="Simulation mode")
    mode.add_argument("--hardware", action="store_true", help="Hardware mode")
    parser.add_argument("--ip", type=str, help="Robot IP (required for --hardware)")

    args = parser.parse_args()

    if args.hardware and not args.ip:
        parser.error("--ip required for --hardware")

    # Create driver (same interface for sim and hardware)
    driver = UR5eDriver(hardware=args.hardware, robot_ip=args.ip)

    try:
        run_interactive(driver)
    except KeyboardInterrupt:
        print("\nInterrupted")
    finally:
        driver.stop()


if __name__ == "__main__":
    main()
