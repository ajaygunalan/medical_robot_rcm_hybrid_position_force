#!/usr/bin/env python3
"""Verify UR5e connection before running control scripts.

Usage:
    python driver/verify_robot_connection.py --ip 169.254.120.1
"""
import argparse
import subprocess
import sys

ROBOT_MODES = {
    0: "DISCONNECTED", 1: "CONFIRM_SAFETY", 2: "BOOTING", 3: "POWER_OFF",
    4: "POWER_ON", 5: "IDLE", 6: "BACKDRIVE", 7: "RUNNING"
}

def main():
    parser = argparse.ArgumentParser(description="Verify UR5e connection")
    parser.add_argument("--ip", required=True, help="Robot IP address")
    args = parser.parse_args()

    # 1. Network ping
    print(f"[1/4] Pinging {args.ip}...", end=" ", flush=True)
    result = subprocess.run(["ping", "-c", "1", "-W", "2", args.ip], capture_output=True)
    if result.returncode != 0:
        print("FAILED - Check ethernet cable")
        sys.exit(1)
    print("OK - Cable connected")

    # 2. RTDE connection
    print("[2/4] Connecting via RTDE...", end=" ", flush=True)
    import rtde_receive
    rtde_r = rtde_receive.RTDEReceiveInterface(args.ip)
    print("OK")

    # 3. Robot power state
    print("[3/4] Checking robot state...", end=" ", flush=True)
    mode = rtde_r.getRobotMode()
    mode_name = ROBOT_MODES.get(mode, "UNKNOWN")
    if mode < 5:
        print(f"NOT READY - Robot is {mode_name}")
        print("      Power on the robot from teach pendant")
        sys.exit(1)
    print(f"OK - Robot is {mode_name}")

    # 4. Visualize in Meshcat
    print("[4/4] Visualizing in Meshcat...", end=" ", flush=True)
    from ur5e_driver import UR5eDriver
    import time

    q = rtde_r.getActualQ()
    driver = UR5eDriver(hardware=False)
    driver._q = q
    driver._plant.SetPositions(driver._plant_ctx, q)
    driver._visualizer.ForcedPublish(driver._vis_ctx)
    print("OK")

    print(f"\n      Meshcat: {driver.meshcat.web_url()}")
    print(f"      Joint angles (deg): {[f'{x*57.3:.1f}' for x in q]}")
    print("\nAll checks passed. Robot ready for control.")
    print("Press [Esc] to exit.\n")

    import select
    import tty
    import termios
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    tty.setcbreak(fd)

    try:
        while True:
            q = rtde_r.getActualQ()
            driver._plant.SetPositions(driver._plant_ctx, q)
            driver._visualizer.ForcedPublish(driver._vis_ctx)
            if select.select([sys.stdin], [], [], 0.1)[0]:
                if sys.stdin.read(1) == '\x1b':
                    break
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

    print("Done.")

if __name__ == "__main__":
    main()
