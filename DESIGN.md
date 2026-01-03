# UR5e Driver Design Document

## Current State (Completed)

### Digital Twin Architecture

The driver uses a **Digital Twin pattern** - an internal Drake plant that mirrors the robot state.

```
┌─────────────────────────────────────────────────────────────┐
│                    APPLICATION LAYER                        │
│  example.py, demo.py                                        │
│  - Does NOT know if running on sim or hardware              │
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
│  Simulation Mode: q += v * dt (kinematic integration)       │
│  Hardware Mode: Reads q from rtde, sends v via speedJ       │
└─────────────────────────────┬───────────────────────────────┘
                              │
               ┌──────────────┴──────────────┐
               ▼                             ▼
       SIMULATION                        HARDWARE
       (internal integration)            (ur_rtde)
```

### Implemented Files

| File | Status | Description |
|------|--------|-------------|
| `driver/ur5e_driver.py` | Done | Digital Twin driver (252 lines) |
| `driver/example.py` | **Needs Update** | Interactive pose control (177 lines) |
| `demo.py` | Done | RCM demo with `--sim` / `--hardware` flags |
| `controller.py` | Unchanged | RCM QP solver (pure math) |

### Usage

```bash
# Simulation
python driver/example.py --sim
python demo.py --sim

# Hardware (same code, different flag)
python driver/example.py --hardware --ip 192.168.1.1
python demo.py --hardware --ip 192.168.1.1
```

---

## Issue: Velocity Control Safety

### Problem

Current `example.py` uses P-control:

```python
error = q_target - q
v_cmd = Kp * error  # DANGEROUS: large error → large velocity
```

When error is large, robot moves too fast. This is unsafe for hardware.

### Desired Behavior

- **Constant, predictable speed** regardless of distance
- **Smooth acceleration/deceleration** at start and end
- **Same behavior** whether moving 0.1 rad or 3.0 rad

---

## Research Summary (Completed)

### Trajectory Generation Options Evaluated

| Approach | Description | Jerk Control | Decision |
|----------|-------------|--------------|----------|
| Drake `FirstOrderHold` | Linear interpolation, constant velocity | No (infinite at transitions) | Too abrupt |
| Drake `CubicShapePreserving` | Cubic spline, smooth velocity/accel | No (but continuous accel) | **RECOMMENDED** |
| Drake `Toppra` | Time-optimal retiming with vel/accel limits | No | Good for strict limits |
| Ruckig (external) | S-curve with jerk limits | Yes | Not needed, adds dependency |

### Why Drake Is Sufficient

1. **`CubicShapePreserving`** provides continuous acceleration (smooth enough for UR5e)
2. **`Toppra`** available if strict velocity/acceleration limits needed later
3. **No external dependencies** - everything in Drake ecosystem
4. **Integrates with existing Digital Twin** architecture

### Drake Trajectory Tools Available

```
PiecewisePolynomial methods:
├── FirstOrderHold()          → Linear, constant velocity (abrupt)
├── CubicShapePreserving()    → Smooth, monotonic (RECOMMENDED)
├── CubicWithContinuousSecondDerivatives() → Smooth accel
└── Cubic() / CubicHermite()  → Specify velocities at knots

Optional post-processing:
└── Toppra → Retime to respect velocity/acceleration limits
```

---

## Solution: Trajectory-Based Control with Drake

### Architecture

```
User selects pose → Generate trajectory → Follow with feedforward + feedback
       │                    │                         │
       ▼                    ▼                         ▼
   q_target         PiecewisePolynomial        v_cmd = v_traj + Kp*(q_traj - q_actual)
                    .CubicShapePreserving()
```

### Velocity Profile Comparison

```
FirstOrderHold (NOT recommended):
Velocity:   ┌────────────────┐
            │                │      ← Instant jump (harsh)
       ─────┘                └─────

CubicShapePreserving (RECOMMENDED):
Velocity:      ╱────────────╲
              ╱              ╲      ← Smooth ramp (gentle)
       ──────╱                ╲──────
```

### Control Law

```python
v_cmd = v_trajectory + Kp * (q_trajectory - q_actual)
```

Where:
- `v_trajectory` = derivative of position trajectory (feedforward)
- `Kp * error` = small correction for tracking (Kp ~ 0.5)
- Error stays small if trajectory is followed well → no velocity spikes

---

## Implementation Plan

### Files to Modify

| File | Changes Required | Effort |
|------|------------------|--------|
| `driver/example.py` | Replace P-control with trajectory-based control | **Main change** |
| `driver/ur5e_driver.py` | None (already supports velocity commands) | No change |

### Changes to `driver/example.py`

**Current flow:**
```
1. User presses key → select target pose
2. Compute error = q_target - q_current
3. v_cmd = Kp * error  ← PROBLEM: velocity proportional to error
4. Send velocity, step
```

**New flow:**
```
1. User presses key → select target pose
2. Generate trajectory: CubicShapePreserving from q_current to q_target
3. Compute duration based on MAX_VELOCITY
4. Follow trajectory:
   - q_des, v_des = sample trajectory at time t
   - v_cmd = v_des + Kp * (q_des - q_actual)  ← Feedforward + feedback
   - Send velocity, step
   - Increment t
5. When t >= duration, trajectory complete
```

### Key Parameters

```python
MAX_VELOCITY = 0.5      # rad/s - safe max joint velocity
MAX_ACCELERATION = 1.0  # rad/s² - for duration calculation (optional)
Kp_TRACKING = 0.5       # Small feedback gain for tracking errors
dt = 0.002              # 500 Hz control loop
```

### Duration Calculation

```python
distance = np.max(np.abs(q_target - q_start))  # Max joint displacement
duration = distance / MAX_VELOCITY              # Time at constant speed
duration = max(duration, 0.5)                   # Minimum 0.5s for short moves
```

---

## Next Steps for Implementation

### Step 1: Update `driver/example.py`

1. Add trajectory generation function using `PiecewisePolynomial.CubicShapePreserving`
2. Replace the control loop to follow trajectory with feedforward + feedback
3. Add duration calculation based on max velocity
4. Keep the keyboard interface unchanged

### Step 2: Test in Simulation

```bash
python driver/example.py --sim
```

Verify:
- Smooth motion between poses
- Consistent speed regardless of distance
- No velocity spikes

### Step 3: Hardware Test (Later)

```bash
python driver/example.py --hardware --ip <robot_ip>
```

---

## References

- [Drake PiecewisePolynomial](https://drake.mit.edu/doxygen_cxx/classdrake_1_1trajectories_1_1_piecewise_polynomial.html)
- [Drake Toppra](https://drake.mit.edu/doxygen_cxx/classdrake_1_1multibody_1_1_toppra.html)
- [MIT Manipulation - Pick and Place](https://manipulation.csail.mit.edu/pick.html)
- [ur_rtde speedJ Documentation](https://sdurobotics.gitlab.io/ur_rtde/)
