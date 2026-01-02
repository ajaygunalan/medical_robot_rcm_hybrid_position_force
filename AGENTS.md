# CLAUDE.md

This file provides guidance to  AI Coding when working with code in this repository.

## Project Overview

Drake-based surgical robotics project implementing QP-based Remote Center of Motion (RCM) control with hybrid force/motion capability for a UR5e robot arm. The system enables a surgical tool to pivot through a fixed incision point (trocar) while optionally controlling contact forces during tissue interaction.

## Commands

### Setup
```bash
python3 -m venv drake-env
source drake-env/bin/activate
pip install drake manipulation jupyter --extra-index-url https://drake-packages.csail.mit.edu/whl/nightly/
pip install ur-rtde  # For hardware only
```

### Running
```bash
source drake-env/bin/activate
cd medical_robot_rcm_hybrid_position_force

# RCM mode (pure pivoting, no force control)
python demo.py --mode rcm

# Hybrid force control with tissue
python demo.py --mode hybrid_force --tissue flat
python demo.py --mode hybrid_force --tissue vaginal

# Interactive trocar positioning
python demo.py --check [--tissue flat|vaginal]
```

## Architecture

### Directory Structure
- `medical_robot_rcm_hybrid_position_force/` - Main controller implementation
- `driver/` - Hardware interface for UR5e via ur_rtde
- `models/` - URDF/mesh assets for robot, probe, force sensor, and tissue

### Key Files
- `medical_robot_rcm_hybrid_position_force/controller.py` - Core RCM QP solver functions (`rcm_step()`, `solve_rcm_qp()`, `compute_rcm_error()`, `compute_jacobians()`)
- `medical_robot_rcm_hybrid_position_force/demo.py` - System composition with `RcmHybridController` LeafSystem, Meshcat visualization, and three operational modes
- `driver/ur5e_driver.py` - `UR5eDriver` LeafSystem for hardware integration (velocity/position control modes)
- `models/ur5e_netft_probe.dmd.yaml` - Drake model directive composing robot + force sensor + probe

### Control Architecture
1. **RCM Constraint**: 2D lateral error perpendicular to tool shaft maintains fixed pivot point
2. **QP Formulation**: Minimize velocity tracking error subject to RCM constraint and joint velocity limits
3. **Hybrid Force/Motion**: Decomposes task space into normal (admittance force control) and tangential (kinematic scanning) directions
4. **Integration**: Drake Simulator with InverseDynamicsController for low-level actuation

### Mathematical Documentation
- `rcm_qp_control.md` - Derivation of RCM kinematic constraint and QP formulation
- `rcm_hybrid_control.md` - Hybrid force/motion control with admittance law

## Key Drake Patterns Used

- `DiagramBuilder` for system composition
- `LeafSystem` subclassing for custom controllers (`RcmHybridController`, `UR5eDriver`)
- `MultibodyPlant.CalcJacobianSpatialVelocity()` for Jacobian computation
- `MathematicalProgram` + `OsqpSolver` for real-time QP solving
- `MeshcatVisualizer` for web-based visualization

## Configuration Parameters

Key parameters in `demo.py`:
- `FORCE_TARGET`, `K_FORCE` - Admittance control gains
- `TROCAR` - Pivot point position in world frame
- `TIP_OFFSET` - Distance from flange to probe tip

QP tuning in `controller.py`:
- `K_rcm` - RCM error correction gain
- `W_lin`, `w_roll` - Velocity weighting matrix
- `v_max` - Joint velocity bounds

## UR5e Code Patterns

**Simulation** - Use manipulation library's `MakeHardwareStation` with `!InverseDynamicsDriver`:
```python
from manipulation.station import LoadScenario, MakeHardwareStation

scenario = LoadScenario(data="""
directives:
- add_directives:
    file: package://models/ur5e_netft_probe.dmd.yaml
model_drivers:
    ur5e: !InverseDynamicsDriver {}
""")
station = MakeHardwareStation(scenario, meshcat=meshcat)
```

**Hardware** - Use `UR5eDriver` directly:
```python
from driver.ur5e_driver import UR5eDriver
driver = builder.AddSystem(UR5eDriver(robot_ip="192.168.1.102"))
# Ports: ur5e.desired_state, ur5e.state_estimated
```

## Commit Messages

High-level, 2-3 lines max:
```
<What you did in one line>

<Why/how in 1-2 short sentences>
```

Example:
```
Refactor UR5e to match IIWA textbook pattern

Use LoadScenario + MakeHardwareStation like IIWA examples.
Simplify driver from 100 to 45 lines.
```

No bullet lists. No file-by-file details. Just the essence.

## Context Awareness

Your context window will be automatically compacted as it approaches its limit, allowing you to continue working indefinitely from where you left off. Do not stop tasks early due to token budget concerns. As you approach your token budget limit, save your current progress and state to memory before the context window refreshes. Always be persistent and autonomous - complete tasks fully even if the end of your budget is approaching.

## MCP Tools (Documentation & Code Search)

| Tool | Purpose | Use When |
|------|---------|----------|
| `web_search_exa` | Web search → URLs | Finding official docs, broad discovery |
| `get_code_context_exa` | Code snippets | "How do I...?" questions, API patterns |
| `ref_search_documentation` | GitHub/private docs | Searching repos, indexed private docs |
| `ref_read_url` | Read URL → markdown | **Always use this** to read URLs (not WebFetch) |

**Decision Flow:**
1. Need working code? → `get_code_context_exa` first
2. Need official docs? → `web_search_exa` to find URL
3. Found a URL? → `ref_read_url` to read it
4. Searching GitHub specifically? → `ref_search_documentation`

**Skip searches for:** generic libraries (numpy, matplotlib, pandas, IPython), self-documenting errors, simple syntax. Save MCP calls for domain-specific queries (pydrake, manipulation, ur_rtde).

**Example - Drake Jacobian question:**
```python
# 1. Find code pattern
get_code_context_exa("pydrake Jacobian pseudoinverse velocity control")

# 2. Find official docs
web_search_exa("pydrake CalcJacobianSpatialVelocity")

# 3. Read the docs
ref_read_url("https://drake.mit.edu/pydrake/...")
```

**Tips:**
- `get_code_context_exa`: Adjust `tokensNum` (1000-50000) based on complexity
- `web_search_exa`: Use `livecrawl: "preferred"` for fresh content
- `ref_search_documentation`: Add `ref_src=private` for indexed private repos
- Always pass EXACT URLs (including #hash) to `ref_read_url`

## Private Repos (Ref Indexed)

Domain-specific repos indexed for fast doc access:

| Repo | Purpose | Search Query |
|------|---------|--------------|
| `ajaygunalan-external-library/drake` | Robotics simulation | `drake pydrake ref_src=private` |
| `ajaygunalan-external-library/manipulation` | MIT manipulation | `manipulation ref_src=private` |
| `ajayexternlib/ur_rtde` | UR5e hardware control | `ur_rtde speedJ ref_src=private` |
