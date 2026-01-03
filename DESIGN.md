# UR5e Driver Design Document

## Status: Implementation Complete ✓

Trajectory-based velocity control is implemented and tested in simulation.

---

## Architecture

### Digital Twin Pattern

```
┌─────────────────────────────────────────────────────────────┐
│                    APPLICATION LAYER                        │
│  example.py, demo.py                                        │
│  - Same code for sim and hardware                           │
│  - Gets plant from driver for Jacobian computation          │
└─────────────────────────────┬───────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                      UR5eDriver                             │
│  ┌───────────────────────────────────────────────────────┐ │
│  │  Internal Drake Plant ("Digital Twin")                 │ │
│  │  - Always exists (sim and hardware)                    │ │
│  │  - Used for Jacobians by controller.py                 │ │
│  │  - Used for Meshcat visualization                      │ │
│  └───────────────────────────────────────────────────────┘ │
│                                                             │
│  Simulation: q += v * dt (kinematic integration)            │
│  Hardware: Reads q from rtde, sends v via speedJ            │
└─────────────────────────────┬───────────────────────────────┘
                              │
               ┌──────────────┴──────────────┐
               ▼                             ▼
       SIMULATION                        HARDWARE
       (internal integration)            (ur_rtde)
```

### Trajectory Control Flow

```
User presses key → Generate trajectory → Follow with feedforward + feedback
       │                    │                         │
       ▼                    ▼                         ▼
   q_target      PiecewisePolynomial         v_cmd = v_des + Kp*(q_des - q_actual)
                 .CubicWithContinuous
                 SecondDerivatives()
```

---

## Files

| File | Status | Description |
|------|--------|-------------|
| `driver/ur5e_driver.py` | ✓ Done | Digital Twin driver |
| `driver/example.py` | ✓ Done | Trajectory-based pose control |
| `demo.py` | ✓ Done | RCM demo with `--sim` / `--hardware` |
| `controller.py` | Unchanged | RCM QP solver (pure math) |

---

## Usage

```bash
# Simulation
python driver/example.py --sim

# Hardware
python driver/example.py --hardware --ip 169.254.120.1
```

Controls: `[a]` pose_a, `[b]` pose_b, `[h]` home, `[e]` equilibrium, `[s]` stop, `[Esc]` exit

---

## What Was Done

### Problem Solved

Old P-control was dangerous:
```python
v_cmd = Kp * (q_target - q)  # Large error → large velocity
```

### Solution Implemented

Trajectory-based control with feedforward + feedback:
```python
# Generate smooth trajectory (rest-to-rest)
traj = PiecewisePolynomial.CubicWithContinuousSecondDerivatives(
    times, positions, zero_vel, zero_vel
)

# Follow with feedforward + small feedback
v_cmd = v_des + Kp * (q_des - q_actual)
```

### Key Parameters (in `driver/example.py`)

```python
MAX_VELOCITY = 0.1   # rad/s - controls speed (lower = slower)
KP_TRACKING = 0.5    # feedback gain
MIN_DURATION = 0.5   # seconds - minimum motion time
DT = 0.002           # 500 Hz control loop
```

---

## Research Summary

### Why Drake (not external libraries)

| Option | Pros | Cons | Decision |
|--------|------|------|----------|
| Drake `CubicWithContinuousSecondDerivatives` | Smooth, no deps, rest-to-rest | No jerk limits | ✓ **Used** |
| Drake `Toppra` | Vel/accel limits | More complex | Available if needed |
| Ruckig (external) | Jerk limits, S-curve | Extra dependency | Not needed |

**Decision:** Stay within Drake ecosystem. Cubic polynomials with zero velocity at endpoints give smooth enough motion for UR5e.

### Velocity Profile

```
Cubic (what we use):
Velocity:      ╱────────────╲
              ╱              ╲      ← Smooth ramp up/down
       ──────╱                ╲──────

Linear (NOT used):
Velocity:   ┌────────────────┐
            │                │      ← Instant jump (harsh)
       ─────┘                └─────
```

---

## Future Work (Optional)

1. **Add Toppra** if strict velocity/acceleration limits needed
2. **Hardware testing** - test on real UR5e
3. **Apply to demo.py** - same pattern for RCM control if needed

---

## References

- [Drake PiecewisePolynomial](https://drake.mit.edu/doxygen_cxx/classdrake_1_1trajectories_1_1_piecewise_polynomial.html)
- [Drake Toppra](https://drake.mit.edu/doxygen_cxx/classdrake_1_1multibody_1_1_toppra.html)
- [ur_rtde Documentation](https://sdurobotics.gitlab.io/ur_rtde/)
