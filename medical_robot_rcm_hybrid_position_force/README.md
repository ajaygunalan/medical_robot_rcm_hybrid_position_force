# RCM QP Controller

QP-based control for surgical tool pivoting through a fixed trocar point.

## Files

| File | Purpose |
|------|---------|
| `rcm_control.py` | Control functions |
| `rcm_demo.py` | UR5e demo |
| `rcm_qp_control.md` | Math derivation |

## Run

```bash
cd rcm_qp
python rcm_demo.py
```

## Usage

```python
from rcm_control import rcm_step

v_opt = rcm_step(plant, ctx, q, trocar_pos, V_des)
q = q + v_opt * dt
```

## Config

In `rcm_demo.py`:
- `TROCAR`: pivot position [0.4, 0, 0.3]
- `TIP_OFFSET`: flange to tip distance
- `DT`: timestep
- `DURATION`: demo length
