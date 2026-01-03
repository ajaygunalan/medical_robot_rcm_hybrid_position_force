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
python driver/example.py --sim

# Hardware (dry-run first)
python driver/example.py --hardware --ip 169.254.120.1
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


## ur_rtde Control Methods Reference

### Motion Commands

| Command | Type | Description | Rate |
|---------|------|-------------|------|
| `moveJ` | Joint space | Move to joint position (trajectory planned internally) | One-shot |
| `moveL` | Cartesian | Move to TCP pose linearly | One-shot |
| `servoJ` | Joint position | Real-time joint position control | 500Hz |
| `servoL` | Cartesian position | Real-time TCP pose control | 500Hz |
| `speedJ` | Joint velocity | Real-time joint velocity control | 500Hz |
| `speedL` | Cartesian velocity | Real-time TCP velocity control | 500Hz |
| `jogStart/jogStop` | Jogging | Manual jogging in tool/base frame | Continuous |
| `movePath` | Path | Execute multi-waypoint path | One-shot |
| `moveUntilContact` | Contact | Move until contact detected | One-shot |

### Force Control

| Command | Description |
|---------|-------------|
| `forceMode` | Hybrid force/position control (500Hz) |
| `forceModeStop` | Exit force mode |
