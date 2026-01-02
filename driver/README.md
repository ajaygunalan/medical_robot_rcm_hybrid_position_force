# UR5e Driver

Hardware interface for UR5e using [ur_rtde](https://sdurobotics.gitlab.io/ur_rtde/) (SDU library).

## Files

| File | Purpose |
|------|---------|
| `ur5e_driver.py` | Drake LeafSystem for hardware control (500Hz) |
| `example.py` | Interactive control with keyboard |

## Usage

```bash
# Simulation
python driver/example.py --sim --control velocity

# Hardware (dry-run first)
python driver/example.py --hardware --control velocity --ip 169.254.120.1
```

## Interactive Controls

```
[a] pose_a   [b] pose_b   [h] home   [e] equilibrium
[s] stop     [Esc] exit
```

## Poses

| Key | Name | Description |
|-----|------|-------------|
| h | home | Like equilibrium but elbow=0° |
| e | equilibrium | Drake default (upright, elbow bent) |
| a | pose_a | UR ROS2 Driver traj0 |
| b | pose_b | UR ROS2 Driver traj1 |

## Protocol

```
--sim:      Start → interactive loop → Esc
--hardware: Read position → dry-run (sim) → [y/N] → interactive hardware
```

## Dependencies

```bash
pip install ur-rtde  # Hardware only
```
