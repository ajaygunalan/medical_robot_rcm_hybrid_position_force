# Convex QP Control for Remote Center of Motion (RCM)

> **The Question:** A surgical tool must pivot through a fixed incision point (trocar). Given a desired tip motion, what joint velocities should we command?

---

## 1. The Problem

In minimally invasive surgery, a straight tool passes through a small incision. The incision point acts as a **pivot** — the tool can slide and rotate through it, but cannot translate sideways.

**The constraint:** The tool shaft must always pass through the trocar point.

**What we control:** A 6-DOF robotic arm holding the tool.

**What we want:** Joint velocities $v \in \mathbb{R}^6$ that:
1. Keep the shaft passing through the trocar (hard constraint)
2. Track a desired tip motion (soft objective)

### DOF Accounting

| | Count |
|---|---|
| Robot joints | 6 |
| RCM constraint (lateral) | −2 |
| **Task DOF** | **4** |

The 4 controllable task DOF are:

| DOF | What it is |
|---|---|
| Insertion | Slide along shaft |
| Pitch | Elevation about trocar |
| Yaw | Azimuth about trocar |
| Roll | Rotation about shaft |

**Key insight:** Pitch and yaw are not independent commands — they are geometrically determined by where the tip is relative to the trocar.

---

## 2. The Solution: A Quadratic Program

At each timestep, we solve a **convex QP** to find the optimal joint velocities:

$$ \min_v \quad \underbrace{\frac{1}{2} v^\top H v + f^\top v}_{\text{cost: tracking error}} \quad \text{subject to} \quad \underbrace{A_{eq} \, v = b_{eq}}_{\text{RCM constraint}}, \quad \underbrace{v_{min} \le v \le v_{max}}_{\text{joint limits}} $$

The solver returns $v^* \in \mathbb{R}^6$ — the **optimal joint velocity command**.

### What Each Term Represents

| Term | Role | Size |
|------|------|------|
| $v$ | Decision variable (joint velocities) | $6 \times 1$ |
| $H, f$ | Quadratic cost (tip tracking error) | $6 \times 6$, $6 \times 1$ |
| $A_{eq}, b_{eq}$ | Equality constraint (RCM) | $2 \times 6$, $2 \times 1$ |
| $v_{min}, v_{max}$ | Joint velocity limits | $6 \times 1$ |

After solving, integrate to get the next joint configuration:

$$ q_{k+1} = q_k + v^* \cdot \Delta t $$

---

## 3. The Control Loop (Roadmap)

Here is the complete algorithm. Each step is explained in subsequent sections.

$$ \boxed{ \begin{aligned}
&\textbf{At each timestep:} \\
&1. \quad \text{Read joints } q \\
&2. \quad \text{Forward kinematics} \rightarrow {}^WR^F, \; {}^Wp^F \\
&3. \quad \text{Compute shaft direction } {}^W\hat{n} \\
&4. \quad \text{Compute projection point } P \text{ and error } e \\
&5. \quad \text{Compute Jacobians } J_P, \; J_T \\
&6. \quad \text{Build constraint: } A_{eq}, \; b_{eq} \\
&7. \quad \text{Build cost: } H, \; f \\
&8. \quad \text{Solve QP} \rightarrow v^* \\
&9. \quad \text{Command: } q \leftarrow q + v^* \Delta t
\end{aligned} } $$

---

## 4. Building the Constraint ($A_{eq}$, $b_{eq}$)

> **What:** Equality constraint matrices $A_{eq}$, $b_{eq}$
>
> **Why:** Hard constraint in QP — forces shaft to pass through trocar
>
> **Needs:** $e$, $B_\perp$ (Section 6), $J_P$ (Section 7)
>
> **Produces:** $A_{eq} \in \mathbb{R}^{2 \times 6}$, $b_{eq} \in \mathbb{R}^{2}$

The RCM constraint says: the shaft must pass through the trocar.

At any instant, there is a point $P$ on the shaft closest to the trocar $C$. The error is:

$$ e = {}^Wp^C - {}^Wp^P $$

where $e$ is purely **lateral** (perpendicular to the shaft). If $|e| = 0$, the constraint is satisfied.

### The Constraint Equation

We want the lateral velocity of $P$ to correct the error:

$$ \underbrace{B_\perp \, J_P}_{A_{eq} \in \mathbb{R}^{2 \times 6}} \, v = \underbrace{K_{rcm} \, B_\perp \, e}_{b_{eq} \in \mathbb{R}^{2}} $$

where:
- $J_P \in \mathbb{R}^{3 \times 6}$ is the Jacobian mapping joint velocities to velocity of point $P$
- $B_\perp \in \mathbb{R}^{2 \times 3}$ extracts the two lateral components (perpendicular to shaft)
- $K_{rcm} > 0$ is the proportional gain for error correction

**Why only 2 rows?** The constraint is 2-dimensional (lateral plane). Motion along the shaft (insertion) remains free. Using a 2×3 basis $B_\perp$ instead of a 3×3 projection matrix avoids rank-deficiency issues in the solver.

---

## 5. Building the Cost ($H$, $f$)

> **What:** Quadratic cost matrices $H$, $f$
>
> **Why:** Soft objective in QP — tracks desired tip motion
>
> **Needs:** $J_T$ (Section 7), desired twist $V_{des}$, weight matrix $Q$
>
> **Produces:** $H \in \mathbb{R}^{6 \times 6}$, $f \in \mathbb{R}^{6}$

We want the tip to track a desired spatial velocity $V_{des} \in \mathbb{R}^6$:

$$ V_{des} = \begin{bmatrix} {}^W\omega^T_{des} \\ {}^W\dot{p}^T_{des} \end{bmatrix} $$

where the top 3 components are angular velocity and bottom 3 are linear velocity.

### The Cost Function

$$ \min_v \; \frac{1}{2} \left( J_T v - V_{des} \right)^\top Q \left( J_T v - V_{des} \right) + \frac{\epsilon}{2} \| v \|^2 $$

where:
- $J_T \in \mathbb{R}^{6 \times 6}$ is the tip spatial Jacobian
- $Q \in \mathbb{R}^{6 \times 6}$ is a positive semidefinite weight matrix
- $\epsilon > 0$ is a regularization term (small, e.g., 0.01)

### Standard QP Form

Expanding into $\frac{1}{2} v^\top H v + f^\top v$:

$$ H = J_T^\top Q \, J_T + \epsilon I_{6 \times 6} $$

$$ f = -J_T^\top Q \, V_{des} $$

### Weight Matrix $Q$

We want to track:
- **Linear velocity** (3 DOF) — primary task
- **Roll** (1 DOF) — rotation about shaft axis

We do **not** penalize pitch/yaw angular velocity — these arise from geometry.

Define the roll projector:

$$ P_{roll} = {}^W\hat{n} \, ({}^W\hat{n})^\top \in \mathbb{R}^{3 \times 3} $$

This extracts only the component of angular velocity about the shaft.

$$ Q = \begin{bmatrix} w_{roll} \, P_{roll} & 0 \\ 0 & w_p \, I_{3 \times 3} \end{bmatrix} $$

where $w_{roll} \ge 0$ weights roll tracking and $w_p > 0$ weights tip position tracking.

**Why regularization?** $\epsilon > 0$ ensures $H$ is strictly positive definite, preventing unbounded solutions near singularities.

---

## 6. Computing the Error

> **What:** Error vector $e$ and lateral basis $B_\perp$
>
> **Why:** Needed for the RCM constraint (Section 4):
> $A_{eq} = B_\perp J_P$, $b_{eq} = K_{rcm} B_\perp e$
>
> **Given:** Joint angles $q$, trocar position ${}^Wp^C$
>
> **Find:** $e \in \mathbb{R}^3$, $B_\perp \in \mathbb{R}^{2 \times 3}$

### Frames

```
W = World frame (fixed)
F = Flange frame (end of robot arm, moves with joints)
T = Tool tip frame (at end of tool, offset from F)
C = Trocar point (fixed in world)
P = Projection point (on shaft, closest to C)
```

### Diagram

```
         Trocar (C)
            *
            |  <-- e (error vector, perpendicular to shaft)
            |
    ========*=====================> Tool Shaft (direction n_hat)
           P
           ^
     Projection point
```

### Step-by-Step Computation

**Step 1 — Shaft Direction**

The shaft axis is the $+z$ axis of the flange frame:

$$ {}^F\hat{n} = \begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix} $$

Transform to world coordinates:

$$ {}^W\hat{n} = {}^WR^F \cdot {}^F\hat{n} $$

where ${}^WR^F(q) \in \mathbb{R}^{3 \times 3}$ is the flange rotation matrix from forward kinematics.

**Step 2 — Vector from Flange to Trocar**

$$ r = {}^Wp^C - {}^Wp^F $$

where ${}^Wp^F(q) \in \mathbb{R}^3$ is the flange position from forward kinematics.

**Step 3 — Projection Distance**

$$ \lambda = r^\top \cdot {}^W\hat{n} $$

This is the signed distance along the shaft from flange origin to point $P$.

**Step 4 — Projection Point Position**

$$ {}^Wp^P = {}^Wp^F + \lambda \cdot {}^W\hat{n} $$

**Step 5 — Error Vector**

$$ e = {}^Wp^C - {}^Wp^P $$

**Property:** By construction, $e \perp {}^W\hat{n}$ always (the error is purely lateral).

### Building the Lateral Basis $B_\perp$

We need an orthonormal basis for the plane perpendicular to ${}^W\hat{n}$. One way:

1. Pick any vector not parallel to $\hat{n}$ (e.g., $[1, 0, 0]^\top$ or $[0, 1, 0]^\top$)
2. $u_1 = \text{normalize}(\hat{n} \times \text{arbitrary})$
3. $u_2 = \hat{n} \times u_1$
4. $B_\perp = \begin{bmatrix} u_1^\top \\ u_2^\top \end{bmatrix} \in \mathbb{R}^{2 \times 3}$

---

## 7. Computing the Jacobians

> **What:** Point Jacobian $J_P$ and Tip Jacobian $J_T$
>
> **Why:**
> - $J_P$ needed for constraint: $A_{eq} = B_\perp J_P$ (Section 4)
> - $J_T$ needed for cost: $H = J_T^\top Q J_T + \epsilon I$ (Section 5)
>
> **Given:** Joint angles $q$, flange Jacobian $J_F$ from robot library
>
> **Find:** $J_P \in \mathbb{R}^{3 \times 6}$, $J_T \in \mathbb{R}^{6 \times 6}$

### Jacobian for Point $P$ ($J_P$)

We need the mapping from joint velocities to velocity of point $P$:

$$ {}^W\dot{p}^P = J_P(q) \cdot v $$

**Approximation (commonly used):** Treat $P$ as rigidly attached to the flange at the current offset:

$$ {}^Wr_{FP} = \lambda \cdot {}^W\hat{n} $$

Using the flange spatial Jacobian $J_F$:

$$ J_F = \begin{bmatrix} J_{\omega F} \\ J_{vF} \end{bmatrix} \in \mathbb{R}^{6 \times 6} $$

where $J_{\omega F}$ is the angular part and $J_{vF}$ is the linear part.

The point Jacobian is:

$$ J_P = J_{vF} - [{}^Wr_{FP}]_\times \cdot J_{\omega F} $$

where $[a]_\times$ is the skew-symmetric matrix such that $[a]_\times b = a \times b$.

**Why is the approximation okay?** We re-solve the QP at each timestep, so small errors from treating $P$ as instantaneously rigid get corrected in the next iteration.

### Jacobian for Tip ($J_T$)

The tip spatial Jacobian maps joint velocities to tip twist:

$$ {}^WV^T = \begin{bmatrix} {}^W\omega^T \\ {}^W\dot{p}^T \end{bmatrix} = J_T(q) \cdot v $$

This is computed similarly to $J_P$, but at the tool tip location instead of point $P$.

---

## 8. Practical Notes

### 8.1 Rank Issues with the Constraint

The projection matrix $P_\perp = I - \hat{n}\hat{n}^\top$ is rank 2, not rank 3. If you use:

$$ A_{eq} = P_\perp J_P \quad \text{(3 rows, rank 2)}$$

some QP solvers will fail or behave poorly due to the rank deficiency.

**Solution:** Use the explicit 2×3 basis $B_\perp$ so that $A_{eq} = B_\perp J_P$ is full row rank.

### 8.2 Why Regularization is Not Optional

Without $\epsilon > 0$:
- Near singularities, $J_T$ becomes rank-deficient
- $H = J_T^\top Q J_T$ becomes singular
- The QP has infinitely many solutions or numerical issues

With $\epsilon > 0$:
- $H$ is always strictly positive definite
- Unique solution exists
- Provides velocity damping

Typical value: $\epsilon \in [0.001, 0.1]$

### 8.3 Weighting Roll Correctly

A common mistake is using diagonal weights on angular velocity in world coordinates:

$$ Q_\omega = \text{diag}(w, w, w) \quad \text{(wrong!)} $$

This accidentally penalizes pitch and yaw, which are geometrically determined by the RCM constraint — not independently controllable.

**Correct approach:** Use the roll projector $P_{roll} = \hat{n}\hat{n}^\top$ so that only rotation about the shaft is penalized.

### 8.4 Exact Jacobian for Point $P$

For strict correctness, $\lambda$ is a function of $q$, and its time derivative contributes to $\dot{p}^P$:

- $\dot{\lambda} = \dot{r}^\top \hat{n} + r^\top \dot{\hat{n}}$
- $\dot{\hat{n}} = \omega^F \times \hat{n}$

In practice, the approximation (Section 7) works well because the QP runs at high frequency and $\lambda$ changes slowly.

### 8.5 Augmented QP for Exact RCM

An alternative formulation treats $\lambda$ as a decision variable:

$$ \text{Decision variables: } [v; \dot{\lambda}] \in \mathbb{R}^7 $$

The constraint ${}^Wp^P = {}^Wp^C$ is then exactly linear in the augmented variables. This keeps the QP convex while being geometrically exact.

---

## 9. Summary

### Key Facts

- A **6-DOF arm** under an **RCM constraint (2 DOF)** has **4 task DOF**: insertion, pitch, yaw, roll
- Pitch and yaw are **geometrically determined** by the tip-to-trocar line
- The RCM constraint is enforced as a **hard equality** on lateral velocity
- Tip tracking is a **soft objective** (quadratic cost)
- The QP is **convex** → fast, globally optimal solution at each timestep
- Regularization ($\epsilon > 0$) is **required** for numerical stability

### Quick Reference

| Question | Answer |
|----------|--------|
| Feasible on 6-DOF arm? | Yes |
| What do we control? | Tip position (3 DOF) + Roll (1 DOF) |
| What is automatic? | Pitch & Yaw (from geometry) |
| Why QP? | Balances hard constraint with soft tracking |
| What is $v^*$? | Optimal joint velocities from solver |
| What are $H, f$? | Quadratic cost matrices (tracking) |
| What are $A_{eq}, b_{eq}$? | Linear equality constraint (RCM) |
| Why $\epsilon > 0$? | Strict convexity, singularity handling |
| Why $B_\perp$ not $P_\perp$? | Full row rank for solver stability |
