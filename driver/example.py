#!/usr/bin/env python3
"""UR5e driver test script with interactive multi-target control.

Interactive Mode:
- Robot starts at equilibrium position
- Press [a]/[b]/[h]/[e] to move to target pose
- Press [s] during motion to stop
- Press [Esc] to exit

Usage:
    # Simulation
    python driver/example.py --sim --control velocity

    # Hardware (dry-run first, then interactive)
    python driver/example.py --hardware --control velocity --ip 169.254.120.1
"""
import argparse
import numpy as np
import sys
import select
import termios
import tty
import time
from pathlib import Path

# Named poses
POSES = {
    # Home: like equilibrium but elbow at 0°
    "home": [0.0, -1.5708, 0.0, -1.5708, -1.5708, 0.0],

    # Equilibrium: Drake default (upright, elbow bent)
    "equilibrium": [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0],

    # Target A from UR ROS2 Driver traj0
    "pose_a": [0.043128, -1.28824, 1.37179, -1.82208, -1.63632, -0.18],

    # Target B from UR ROS2 Driver traj1
    "pose_b": [0.30493, -0.982258, 0.955637, -1.48215, -1.72737, 0.204445],
}

# Key mappings for interactive control
KEY_MAP = {'a': 'pose_a', 'b': 'pose_b', 'h': 'home', 'e': 'equilibrium'}

# Very conservative safety parameters for testing
MAX_VELOCITY = 0.05     # rad/s (~3 deg/s) - very slow for safety
ACCELERATION = 0.1      # rad/s^2
CONTROL_DT = 0.002      # 500Hz control loop

# Path to models directory
MODELS_DIR = Path(__file__).parent.parent / 'models'


# =============================================================================
# KEYBOARD INPUT UTILITIES (Unix)
# =============================================================================

def getch_blocking():
    """Get single character without waiting for Enter (blocking)."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


def getch_nonblocking():
    """Get character if available, else return None (non-blocking)."""
    if select.select([sys.stdin], [], [], 0)[0]:
        return getch_blocking()
    return None


def _compute_motion(q_current, q_target):
    """Compute motion parameters.

    Returns:
        diff: Joint position difference array
        max_diff: Maximum absolute difference (rad)
        duration: Motion duration (s)
        steps: Number of control steps at 500Hz
    """
    diff = np.array(q_target) - np.array(q_current)
    max_diff = np.max(np.abs(diff))
    duration = max_diff / MAX_VELOCITY if max_diff > 0.001 else 0.5
    steps = int(duration / CONTROL_DT)
    return diff, max_diff, duration, steps



def create_visualization_diagram(meshcat, initial_pose):
    """Create a simple diagram for visualization only (no simulation).

    Returns plant, visualizer, context components for updating visualization.
    """
    from pydrake.all import (
        DiagramBuilder, AddMultibodyPlantSceneGraph, Parser, MeshcatVisualizer,
    )

    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)

    # Load UR5e model
    parser = Parser(plant)
    parser.package_map().Add("ur5e_description", str(MODELS_DIR))
    parser.AddModels(str(MODELS_DIR / 'ur5e.urdf'))
    plant.Finalize()

    # Add visualizer
    visualizer = MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)

    diagram = builder.Build()
    context = diagram.CreateDefaultContext()

    # Set initial position
    plant_context = plant.GetMyContextFromRoot(context)
    plant.SetPositions(plant_context, initial_pose)

    # Get visualizer context
    vis_context = visualizer.GetMyContextFromRoot(context)

    return plant, visualizer, context, plant_context, vis_context


def _execute_motion_sim(q_start, q_end, duration, plant, plant_context, visualizer, vis_context):
    """Execute motion with keyboard stop check.

    Returns:
        stopped: True if motion was stopped by user, False if completed
        q_current: Final position
    """
    dt = 0.01  # 100Hz visualization update
    steps = int(duration / dt)

    # Keep terminal in cbreak mode during motion for immediate key detection
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)  # Chars available immediately, Ctrl-C still works

        for i in range(steps + 1):
            # Linear interpolation
            alpha = i / max(steps, 1)
            q = q_start + alpha * (q_end - q_start)

            plant.SetPositions(plant_context, q)
            visualizer.ForcedPublish(vis_context)

            # Check for stop request (non-blocking)
            if select.select([sys.stdin], [], [], 0)[0]:
                key = sys.stdin.read(1)
                if key == 's':
                    return True, np.array(q)

            time.sleep(dt)

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    return False, np.array(q_end)


def run_interactive_simulation(control_mode, meshcat=None):
    """Interactive simulation with keyboard control.

    Args:
        control_mode: 'velocity' or 'position' (both use same trajectory here)
        meshcat: Optional existing Meshcat instance to reuse
    """
    from pydrake.all import (
        DiagramBuilder, AddMultibodyPlantSceneGraph, Parser,
        MeshcatVisualizer, StartMeshcat,
    )

    print(f"\n=== INTERACTIVE SIMULATION ({control_mode.upper()}) ===")
    if meshcat is None:
        meshcat = StartMeshcat()

    # Build visualization-only diagram
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
    parser = Parser(plant)
    parser.package_map().Add("ur5e_description", str(MODELS_DIR))
    parser.AddModels(str(MODELS_DIR / 'ur5e.urdf'))
    plant.Finalize()

    visualizer = MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
    diagram = builder.Build()

    context = diagram.CreateDefaultContext()
    plant_context = plant.GetMyContextFromRoot(context)
    vis_context = visualizer.GetMyContextFromRoot(context)

    # Start at equilibrium
    q_current = np.array(POSES["equilibrium"])
    current_name = "equilibrium"
    plant.SetPositions(plant_context, q_current)
    visualizer.ForcedPublish(vis_context)

    print(f"Meshcat: {meshcat.web_url()}")
    print("\nControls:")
    print("  [a] pose_a   [b] pose_b   [h] home   [e] equilibrium")
    print("  [s] stop during motion   [Esc] exit")
    print("\n[Robot visible at equilibrium position]")

    while True:
        print(f"\nAt: {current_name} | Press target key...")
        key = getch_blocking()

        if key == '\x1b':  # Escape
            print("Exiting...")
            break

        if key not in KEY_MAP:
            print(f"Unknown key: {repr(key)}")
            continue

        target_name = KEY_MAP[key]
        q_target = np.array(POSES[target_name])

        if np.allclose(q_current, q_target, atol=0.01):
            print(f"Already at {target_name}")
            continue

        # Compute motion parameters
        diff, max_diff, duration, _ = _compute_motion(q_current, q_target)
        print(f"Moving to {target_name}... ({duration:.1f}s, press 's' to stop)")

        # Execute motion with stop checking
        stopped, q_current = _execute_motion_sim(
            q_current, q_target, duration,
            plant, plant_context, visualizer, vis_context
        )

        if stopped:
            current_name = "stopped"
            print(f"Stopped at {np.round(q_current, 3).tolist()}")
        else:
            current_name = target_name
            print(f"Reached {target_name}")

    return meshcat


def _execute_motion_hw(rtde_c, rtde_r, q_target, control_mode,
                       plant=None, plant_context=None, visualizer=None, vis_context=None):
    """Execute hardware motion with keyboard stop check using async moveJ.

    Returns:
        stopped: True if motion was stopped by user, False if completed
        q_current: Final position
    """
    q_target_list = q_target.tolist()

    # Use async moveJ for interruptible motion
    if control_mode == "velocity":
        # For velocity mode, use moveJ with async=True (internally uses velocity profile)
        rtde_c.moveJ(q_target_list, MAX_VELOCITY, ACCELERATION, True)  # async=True
    else:
        # Position mode - same moveJ but conceptually tracking position
        rtde_c.moveJ(q_target_list, MAX_VELOCITY, ACCELERATION, True)  # async=True

    # Monitor motion progress and check for stop
    while True:
        # Check if motion is still running
        try:
            progress = rtde_c.getAsyncOperationProgressEx()
            if not progress.isAsyncOperationRunning():
                break
        except AttributeError:
            # Fallback if getAsyncOperationProgressEx not available
            q_now = np.array(rtde_r.getActualQ())
            if np.allclose(q_now, q_target, atol=0.01):
                break

        # Update visualization
        if plant is not None and visualizer is not None:
            q_now = np.array(rtde_r.getActualQ())
            plant.SetPositions(plant_context, q_now)
            visualizer.ForcedPublish(vis_context)

        # Check for stop request
        key = getch_nonblocking()
        if key == 's':
            rtde_c.stopJ(2.0)  # Decelerate at 2 rad/s²
            time.sleep(0.2)  # Wait for stop to take effect
            q_final = np.array(rtde_r.getActualQ())
            return True, q_final

        time.sleep(0.05)  # 20Hz check rate

    q_final = np.array(rtde_r.getActualQ())
    return False, q_final


def run_interactive_hardware(control_mode, robot_ip,
                             plant=None, plant_context=None, visualizer=None, vis_context=None):
    """Interactive hardware control with keyboard.

    Args:
        control_mode: 'velocity' or 'position'
        robot_ip: Robot IP address
        plant, plant_context, visualizer, vis_context: Optional viz components
    """
    from rtde_control import RTDEControlInterface
    from rtde_receive import RTDEReceiveInterface

    print("\n=== CONNECTING TO ROBOT ===")
    rtde_c = RTDEControlInterface(robot_ip)
    rtde_r = RTDEReceiveInterface(robot_ip)

    # Read current position
    q_current = np.array(rtde_r.getActualQ())
    current_name = "hardware_start"

    # Update visualization if available
    if plant is not None and visualizer is not None:
        plant.SetPositions(plant_context, q_current)
        visualizer.ForcedPublish(vis_context)

    print(f"Current position: {np.round(q_current, 3).tolist()}")
    print("\nControls:")
    print("  [a] pose_a   [b] pose_b   [h] home   [e] equilibrium")
    print("  [s] stop during motion   [Esc] exit")

    try:
        while True:
            print(f"\nAt: {current_name} | Press target key...")
            key = getch_blocking()

            if key == '\x1b':  # Escape
                print("Exiting...")
                break

            if key not in KEY_MAP:
                print(f"Unknown key: {repr(key)}")
                continue

            target_name = KEY_MAP[key]
            q_target = np.array(POSES[target_name])

            if np.allclose(q_current, q_target, atol=0.01):
                print(f"Already at {target_name}")
                continue

            # Compute motion parameters
            diff, max_diff, duration, _ = _compute_motion(q_current, q_target)
            print(f"Moving to {target_name}... ({duration:.1f}s, press 's' to stop)")

            # Execute motion with stop checking
            stopped, q_current = _execute_motion_hw(
                rtde_c, rtde_r, q_target, control_mode,
                plant, plant_context, visualizer, vis_context
            )

            if stopped:
                current_name = "stopped"
                print(f"Stopped at {np.round(q_current, 3).tolist()}")
            else:
                current_name = target_name
                print(f"Reached {target_name}")

    except Exception as e:
        print(f"ERROR: {e}")
        rtde_c.stopJ(2.0)
    finally:
        rtde_c.stopScript()


def run_hardware_with_dryrun(control_mode, robot_ip):
    """Hardware mode with MANDATORY dry-run first and live visualization.

    Protocol:
    1. Read hardware position and show in Meshcat
    2. Run interactive simulation (dry-run)
    3. Ask user if they want to proceed to hardware
    4. Run interactive hardware control
    """
    from pydrake.all import StartMeshcat

    try:
        from rtde_receive import RTDEReceiveInterface
    except ImportError:
        print("ERROR: ur_rtde not installed. Run: pip install ur-rtde")
        return

    # === STEP 1: READ HARDWARE POSITION ===
    print("\n=== READING HARDWARE POSITION ===")
    print(f"Connecting to robot at {robot_ip}...")

    rtde_r = RTDEReceiveInterface(robot_ip)
    q_hardware = np.array(rtde_r.getActualQ())
    rtde_r.disconnect()

    print(f"Hardware position: {np.round(q_hardware, 3).tolist()}")

    # Create visualization diagram
    print("\nInitializing Meshcat visualization...")
    meshcat = StartMeshcat()
    plant, visualizer, context, plant_context, vis_context = create_visualization_diagram(
        meshcat, q_hardware)

    # Show hardware position in Meshcat
    visualizer.ForcedPublish(vis_context)
    print(f"Meshcat URL: {meshcat.web_url()}")
    print("\n[Robot visible at REAL hardware position]")

    # === STEP 2: MANDATORY DRY-RUN (INTERACTIVE SIMULATION) ===
    input("\nPress Enter to start dry-run (interactive simulation)...")

    print("\n=== DRY-RUN: INTERACTIVE SIMULATION ===")
    print("Test your motion sequence in simulation first.")
    run_interactive_simulation(control_mode, meshcat)

    # === STEP 3: ASK TO PROCEED TO HARDWARE ===
    response = input("\nReady to execute on REAL HARDWARE? [y/N]: ")
    if response.lower() != 'y':
        print("Aborted. Hardware motion not executed.")
        return

    # === STEP 4: INTERACTIVE HARDWARE CONTROL ===
    print("\n=== EXECUTING ON HARDWARE ===")
    run_interactive_hardware(control_mode, robot_ip,
                             plant, plant_context, visualizer, vis_context)


def main():
    parser = argparse.ArgumentParser(
        description="UR5e interactive control with keyboard",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Interactive Control:
  Robot starts at equilibrium position. Use keyboard to select targets.

  Keys:
    [a] pose_a   [b] pose_b   [h] home   [e] equilibrium
    [s] stop during motion   [Esc] exit

Examples:
  # Simulation
  python driver/example.py --sim --control velocity

  # Hardware (dry-run first, then interactive)
  python driver/example.py --hardware --control velocity --ip 169.254.120.1

Poses:
  home        = [0, 0, 0, 0, 0, 0] (all zeros)
  equilibrium = [0, -90°, 90°, -90°, -90°, 0] (Drake default)
  pose_a      = UR ROS2 Driver traj0 target
  pose_b      = UR ROS2 Driver traj1 target
"""
    )

    # Mode selection (mutually exclusive, required)
    mode_group = parser.add_mutually_exclusive_group(required=True)
    mode_group.add_argument("--sim", action="store_true",
                           help="Interactive simulation mode")
    mode_group.add_argument("--hardware", action="store_true",
                           help="Hardware mode (dry-run first)")

    # Control mode (REQUIRED)
    parser.add_argument("--control", type=str, choices=["velocity", "position"],
                       required=True, help="Control mode: velocity or position (REQUIRED)")

    # Hardware options (required for --hardware, validated below)
    parser.add_argument("--ip", type=str,
                       help="Robot IP address (REQUIRED for --hardware)")

    args = parser.parse_args()

    # Validate: --ip required for --hardware
    if args.hardware and not args.ip:
        parser.error("--ip is required for --hardware mode")

    print(f"Mode: {'SIMULATION' if args.sim else 'HARDWARE'}")
    print(f"Control: {args.control.upper()}")

    if args.sim:
        run_interactive_simulation(args.control)
    else:
        run_hardware_with_dryrun(args.control, args.ip)


if __name__ == "__main__":
    main()
