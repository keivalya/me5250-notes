# Modern Robotics — Chapter 2: Configuration Space
## Exam Preparation Notes

---

# Table of Contents
1. [Section 2.1 — Degrees of Freedom of a Rigid Body](#section-21)
2. [Section 2.2 — Degrees of Freedom of a Robot: Grübler's Formula](#section-22)
3. [Section 2.3 — C-Space Topology and Representation](#section-23)
4. [Section 2.4 — Configuration and Velocity Constraints](#section-24)
5. [Section 2.5 — Task Space and Workspace](#section-25)
6. [Quick-Reference Summary Table](#summary)

---

# Section 2.1 — Degrees of Freedom of a Rigid Body {#section-21}

## Core Concept

The **configuration** of a robot is a complete specification of the position of every point of the robot. The minimum number of real-valued coordinates needed to represent the configuration is the number of **degrees of freedom (DOF)**.

## Fundamental DOF Rule

$$\text{DOF} = \text{(number of variables)} - \text{(number of independent constraints)}$$

Equivalently:

$$\text{DOF} = \text{(sum of freedoms of points/bodies)} - \text{(independent constraints)}$$

## Proof: Planar Rigid Body has 3 DOF

Choose three non-collinear points A, B, C on a coin lying on a table.

**Step 1:** If A, B, C could be placed independently, there would be **6 variables**: $(x_A, y_A, x_B, y_B, x_C, y_C)$.

**Step 2:** A rigid body imposes distance constraints:
- $d(A,B) = d_{AB}$ → **1 independent constraint** (removes 1 DOF: 6 → 5)
- $d(A,C) = d_{AC}$ → **1 independent constraint** (removes 1 DOF: 5 → 4)
- $d(B,C) = d_{BC}$ → **REDUNDANT** — once A and B are placed and heads/tails is chosen, C is fully determined

**Step 3:**

$$\text{DOF} = 6 - 2 = 3 \quad \checkmark$$

The third constraint $d(B,C)$ is redundant because the intersection of two circles yields exactly two points (heads or tails), so C is already determined.

## Proof: Spatial Rigid Body has 6 DOF

For three points in 3D, each has 3 coordinates → **9 variables** total.

| Point | Freedoms Added | Constraints |
|-------|---------------|-------------|
| A | 3 (place anywhere) | 0 |
| B | 3 − 1 = **2** | $d(A,B)=d_{AB}$ restricts B to sphere around A |
| C | 3 − 2 = **1** | $d(A,C)=d_{AC}$ and $d(B,C)=d_{BC}$ restrict C to a circle |

$$\text{DOF} = 3 + 2 + 1 = 6 \quad \checkmark$$

> **Key Results:**
> - **Planar rigid body:** DOF = 3 (2 translational + 1 rotational)
> - **Spatial rigid body:** DOF = 6 (3 translational + 3 rotational)

---

## Worked Examples

### ▶ Easy — A rigid rod in a plane

**Question:** A rigid rod lies in the $xy$-plane. Its two endpoints can move freely in the plane. How many DOF does the rod have? Verify by listing the generalized coordinates.

<details>
<summary><b>Solution</b></summary>

**Method 1 (direct count):**
- Place endpoint A at $(x_A, y_A)$ — 2 free variables.
- Endpoint B is constrained to a circle of radius $L$ around A — 1 additional variable (angle $\phi_{AB}$).
- **DOF = 3**

**Method 2 (variables − constraints):**
- Variables: $(x_A, y_A, x_B, y_B)$ = 4 variables
- Constraints: $d(A,B) = L$ → 1 independent constraint
- DOF = 4 − 1 = **3** ✓

**Generalized coordinates:** $(x_A, y_A, \phi)$ where $\phi$ is the angle the rod makes with the x-axis. This is exactly the same as a general planar rigid body: 3 DOF = 2 translational + 1 rotational. ✓

</details>

---

### ▶ Medium — Rod pinned at one end

**Question:** A rigid rod of length $L$ lies in a plane. One endpoint A is pinned to the origin (it cannot translate). How many DOF does the rod have? What is the topology of its C-space?

<details>
<summary><b>Solution</b></summary>

Free planar rigid body: 3 DOF.

Constraints from pinning A at origin:
- $x_A = 0$ → 1 constraint
- $y_A = 0$ → 1 constraint

These are independent (they constrain different coordinates).

$$\text{DOF} = 3 - 2 = 1$$

The single DOF is the angle $\theta \in [0, 2\pi)$ that the rod makes with the x-axis. Since $\theta$ wraps around (0 and $2\pi$ are the same configuration), the **C-space is $S^1$** (a circle), not a line interval.

**Intuition:** A pinned rod is simply a revolute joint — it rotates about the pin with exactly 1 DOF. ✓

</details>

---

### ▶ Hard — Two points of a 3D rigid body constrained to a plane

**Question:** A rigid body moves freely in 3D space. Two distinct, non-coincident points A and B on the body are both constrained to lie in the plane $z = 0$. How many DOF does the body have? Identify the physical interpretation of each DOF.

<details>
<summary><b>Solution</b></summary>

Free spatial rigid body: 6 DOF.

**Constraints:**
- $z_A = 0$ → 1 constraint
- $z_B = 0$ → 1 constraint

Are they independent? Yes — $z_A = 0$ says nothing about $z_B$ (they constrain different points), so both constraints are independent.

$$\text{DOF} = 6 - 2 = 4$$

**Physical interpretation of the 4 DOF:**
1. **Translation in x** — slide body along x in the $z=0$ plane
2. **Translation in y** — slide body along y in the $z=0$ plane
3. **Rotation about z-axis** — spin body in the $z=0$ plane
4. **Rotation about the line AB** (which lies in $z=0$) — this lifts the rest of the body out of $z=0$ while keeping A and B on it, like a flap hinge

**Verification:** After constraining A to $z=0$, the body still has 5 DOF. The additional constraint $z_B = 0$ removes 1 more (the out-of-plane tilt), giving 4 DOF. ✓

</details>

---

# Section 2.2 — Degrees of Freedom of a Robot: Grübler's Formula {#section-22}

## Joint Types Summary

| Joint | Symbol | DOF $f$ | Constraints (planar) | Constraints (spatial) |
|-------|--------|---------|---------------------|-----------------------|
| Revolute | R | 1 | 2 | 5 |
| Prismatic | P | 1 | 2 | 5 |
| Helical | H | 1 | N/A | 5 |
| Cylindrical | C | 2 | N/A | 4 |
| Universal | U | 2 | N/A | 4 |
| Spherical | S | 3 | N/A | 3 |

Note: $f_i + c_i = m$ for every joint $i$, where $m = 3$ (planar) or $m = 6$ (spatial).

## Grübler's Formula

For a mechanism with $N$ links (ground included), $J$ joints, $m$ DOF of a free rigid body, and joint $i$ having $f_i$ DOF:

$$\boxed{\text{dof} = m(N - 1 - J) + \sum_{i=1}^{J} f_i}$$

- **Planar:** $m = 3$
- **Spatial:** $m = 6$

## Derivation

1. Start with $N - 1$ moving links (ground is fixed), each with $m$ free DOF → total free DOF $= m(N-1)$
2. Each joint $i$ removes $c_i = m - f_i$ constraints
3. Therefore:

$$\text{dof} = m(N-1) - \sum c_i = m(N-1) - \sum(m - f_i) = m(N-1-J) + \sum f_i \quad \checkmark$$

> ⚠️ **Important Caveat:** Grübler's formula assumes all joint constraints are **independent**. When constraints are redundant (e.g., parallelogram linkage), the formula may yield an incorrect (usually lower) DOF count. Always verify with physical intuition.

---

## Worked Examples

### ▶ Easy — Planar 3R Serial Chain

**Question:** A planar robot has 3 revolute joints in series (3R arm). The base is grounded. Use Grübler's formula to find the number of DOF.

<details>
<summary><b>Solution</b></summary>

**Identify parameters:**
- $N = 4$ (3 moving links + 1 ground link)
- $J = 3$ (three revolute joints)
- $f_i = 1$ for each revolute joint
- $m = 3$ (planar mechanism)

**Apply Grübler's formula:**

$$\text{dof} = 3(4 - 1 - 3) + 3(1) = 3(0) + 3 = \boxed{3} \checkmark$$

**Intuition:** A 3R planar arm has 3 independent joint angles $\theta_1, \theta_2, \theta_3$, giving 3 DOF as expected. ✓

</details>

---

### ▶ Medium — Planar Five-Bar Linkage

**Question:** A planar five-bar linkage consists of 5 links (one is ground) connected in a single closed loop by 5 revolute joints. Find its DOF using Grübler's formula. What does this DOF count mean physically?

<details>
<summary><b>Solution</b></summary>

**Identify parameters:**
- $N = 5$ (4 moving links + 1 ground)
- $J = 5$ (five revolute joints)
- $f_i = 1$ for all $i$
- $m = 3$ (planar)

**Apply Grübler's formula:**

$$\text{dof} = 3(5 - 1 - 5) + 5(1) = 3(-1) + 5 = -3 + 5 = \boxed{2} \checkmark$$

**Physical meaning:** The five-bar linkage has 2 DOF — if you actuate two joints (typically the two joints attached to ground), the entire mechanism is fully determined. This makes it a popular 2-DOF parallel mechanism for pick-and-place tasks. ✓

</details>

---

### ▶ Hard — Stewart–Gough Platform (6-DOF Parallel Robot)

**Question:** The Stewart–Gough platform consists of a fixed lower platform (ground) and a moving upper platform connected by 6 identical **UPS** legs (Universal–Prismatic–Spherical). Each leg has one U joint ($f=2$), one P joint ($f=1$), and one S joint ($f=3$). Total links = 14 (2 platforms + 6×2 leg links). Verify using Grübler's formula that the platform has 6 DOF.

<details>
<summary><b>Solution</b></summary>

**Count carefully:**
- Links: 2 platforms + $6 \times 2$ leg-links = **14 total** → $N = 14$
- Joints: 6 U joints + 6 P joints + 6 S joints = **18 total** → $J = 18$

**Sum of joint freedoms:**

$$\sum f_i = 6 \times f_U + 6 \times f_P + 6 \times f_S = 6(2) + 6(1) + 6(3) = 12 + 6 + 18 = 36$$

**Apply spatial Grübler ($m = 6$):**

$$\text{dof} = 6(14 - 1 - 18) + 36 = 6(-5) + 36 = -30 + 36 = \boxed{6} \checkmark$$

**Physical significance:** The 6 DOF match the 6 DOF of a free rigid body (3 translational + 3 rotational), meaning the moving platform can achieve any pose in space within its workspace. This makes the Stewart–Gough platform ideal for flight simulators. ✓

</details>

---

# Section 2.3 — C-Space Topology and Representation {#section-23}

## Topological Equivalence

Two spaces are **topologically equivalent** if one can be continuously deformed into the other without cutting or gluing. Topology is an intrinsic property — independent of coordinate choice.

A sphere and a football are topologically equivalent (both can be stretched into each other). A sphere and a plane are **not** (you must cut the sphere to flatten it).

## Key Topological Spaces

| Space | Notation | Description | Robot Example |
|-------|----------|-------------|---------------|
| Line | $\mathbb{R}^1$ | Euclidean 1D | Prismatic joint position |
| $n$-D Euclidean | $\mathbb{R}^n$ | Flat $n$-D | 3D position: $\mathbb{R}^3$ |
| Circle | $S^1$ | 1D sphere | Single revolute joint |
| 2-Sphere | $S^2$ | Surface of 3D ball | Spherical pendulum |
| Torus | $T^2 = S^1 \times S^1$ | 2R arm C-space | Two revolute joints |
| $n$-Torus | $T^n$ | $n$ copies of $S^1$ | $n$R robot (no limits) |

> ⚠️ Note: $S^1 \times S^1 = T^2$ is a **torus**, NOT $S^2$ (a sphere). These are topologically distinct.

## Explicit vs. Implicit Representation

### Explicit Parametrization
Use exactly $n$ coordinates for an $n$-DOF space.

- **Example:** $(x, y, \theta)$ for a planar rigid body.
- **Advantage:** Minimum number of coordinates.
- **Disadvantage:** Singularities may exist (e.g., latitude–longitude at the poles).

### Implicit Representation
Embed the $n$-DOF space in a higher-dimensional Euclidean space and impose constraint equations.

- **Example:** Use a $3 \times 3$ rotation matrix $R$ (9 numbers) subject to 6 orthonormality constraints for the 3 rotational DOF.
- **Advantage:** Singularity-free, enables linear algebra.
- **Disadvantage:** More variables than DOF.

**Verification for SO(3):**
$$\text{Variables: } 9 \quad \text{Constraints from } R^T R = I: 6 \quad \text{DOF} = 9 - 6 = 3 \checkmark$$

## Singularities of Representation

A **singularity** in a parametrization occurs when the coordinates change rapidly or become undefined for a smoothly moving point. For example, longitude is undefined at the poles of a sphere — these are singularities of the latitude–longitude representation, **not** of the sphere itself.

**Solutions:**
1. Use multiple overlapping coordinate charts (an **atlas**), switching away from singularities as needed.
2. Use an **implicit representation**, which is globally singularity-free.

---

## Worked Examples

### ▶ Easy — C-Space Topology of a 2R Robot Arm

**Question:** A planar 2R robot arm has two revolute joints with no joint limits. (a) What is the topology of the C-space? (b) How many real numbers are needed to represent a configuration?

<details>
<summary><b>Solution</b></summary>

**(a)** Each revolute joint angle $\theta_i$ lives on a circle $S^1$ (since $\theta$ and $\theta + 2\pi$ represent the same configuration). With no joint limits, the two angles are completely independent:

$$\text{C-space} = S^1 \times S^1 = T^2 \quad \text{(a 2D torus)}$$

**(b)** Two real numbers are needed: $(\theta_1, \theta_2) \in [0, 2\pi) \times [0, 2\pi)$.

This is an explicit parametrization. The torus has no singularity issues with this standard representation (the identification $0 \equiv 2\pi$ at the boundary is a topological fact, not a computational singularity). ✓

</details>

---

### ▶ Medium — C-Space of a Planar Mobile Robot with a 1R Arm

**Question:** A mobile robot chassis moves freely on an infinite plane. A single revolute joint arm is mounted on the chassis. (a) Describe the C-space topology. (b) State the number of DOF and provide an explicit coordinate representation.

<details>
<summary><b>Solution</b></summary>

**(a) Decompose the system:**

| Component | Space | Reason |
|-----------|-------|--------|
| Chassis position $(x, y)$ | $\mathbb{R}^2$ | Unrestricted translation on plane |
| Chassis orientation $\phi$ | $S^1$ | Wraps around at $2\pi$ |
| Arm joint angle $\theta$ | $S^1$ | Wraps around at $2\pi$ |

$$\text{C-space} = \mathbb{R}^2 \times S^1 \times S^1 = \mathbb{R}^2 \times T^2$$

**(b) DOF** $= 2 + 1 + 1 = 4$.

**Explicit coordinates:** $(x, y, \phi, \theta)$ where $x, y \in \mathbb{R}$ and $\phi, \theta \in [0, 2\pi)$. ✓

</details>

---

### ▶ Hard — Implicit vs. Explicit Representation of a Rigid Body in 3D

**Question:** A rigid body moves freely in 3D space. (a) State the topology of the C-space. (b) Give the implicit representation using a rotation matrix + position vector; count variables and constraints to verify 6 DOF. (c) Explain why the rotation matrix is preferred over roll-pitch-yaw angles for computation.

<details>
<summary><b>Solution</b></summary>

**(a) Topology:**
- Position of one body-fixed point: $\mathbb{R}^3$
- 2 angles for orientation of one axis: $S^2$
- 1 angle for rotation about that axis: $S^1$

$$\text{C-space} = \mathbb{R}^3 \times S^2 \times S^1 \quad \text{(6 dimensional)}$$

**(b) Implicit representation via homogeneous transformation $T \in SE(3)$:**

$$T = \begin{bmatrix} R & p \\ 0 & 1 \end{bmatrix}, \quad R \in \mathbb{R}^{3 \times 3}, \quad p \in \mathbb{R}^3$$

| | Count |
|--|--|
| Variables: 9 entries of $R$ + 3 entries of $p$ | **12 total** |
| Constraints from $R^T R = I$: 3 unit-length + 3 orthogonality | **6 independent** |
| $\det(R) = +1$: follows from the 6 above (dependent) | — |
| **Effective DOF** $= 12 - 6$ | **= 6** ✓ |

**(c) Roll-pitch-yaw (RPY) angles** use only 3 parameters — the minimum — but encounter **singularities (gimbal lock)** at certain orientations (e.g., pitch = $\pm 90°$). Near a singularity, small physical rotations produce large or undefined changes in RPY angles, making numerical computations unreliable.

The **rotation matrix** $R$ is **singularity-free**: any smooth rotation of the body is represented by a smoothly changing $R$. It also enables computations via standard linear algebra (matrix multiplication for composing rotations). The cost — using 9 numbers instead of 3 — is a worthwhile trade-off. ✓

</details>

---

# Section 2.4 — Configuration and Velocity Constraints {#section-24}

## Holonomic Constraints

For a closed-chain robot with joint vector $\theta \in \mathbb{R}^n$, **loop-closure equations** of the form:

$$g(\theta) = 0 \quad \left( g: \mathbb{R}^n \to \mathbb{R}^k, \; k \text{ independent equations} \right)$$

are called **holonomic constraints**. They reduce the dimension of the reachable C-space by $k$.

**Differentiating** $g(\theta(t)) = 0$ with respect to time gives the corresponding velocity constraint:

$$\frac{\partial g}{\partial \theta}(\theta)\, \dot{\theta} = 0 \quad \Longrightarrow \quad A(\theta)\dot{\theta} = 0$$

Since this came from differentiating $g(\theta) = 0$, it is **integrable** — the velocity constraint can be integrated back to a configuration constraint.

## Pfaffian Constraints and Nonholonomic Constraints

A **Pfaffian constraint** has the form $A(\theta)\dot{\theta} = 0$.

| Type | Condition | Effect on C-space |
|------|-----------|-------------------|
| **Holonomic** | $A(\theta) = \partial g / \partial \theta$ for some $g$ | Reduces **dimension of reachable C-space** |
| **Nonholonomic** | No such $g$ exists | Reduces **feasible velocity dimension only**; C-space dimension unchanged |

> **Key insight:** A nonholonomic system can still reach any point in its full C-space — the constraints only restrict *how fast* it can move in each direction at each instant, not *where* it can ultimately go.

## Integrability Test

For a single Pfaffian constraint $A(q)\dot{q} = 0$ with $A = [A_1, A_2, \ldots, A_n]$, it is **integrable (holonomic)** if and only if:

$$\frac{\partial A_i}{\partial q_j} = \frac{\partial A_j}{\partial q_i} \quad \text{for all } i, j$$

This is the condition for the 1-form $A_1 dq_1 + \cdots + A_n dq_n$ to be **exact** (equal to $dg$ for some scalar function $g$).

---

## Worked Examples

### ▶ Easy — Loop-Closure Equations of a Four-Bar Linkage

**Question:** The planar four-bar linkage has loop-closure equations:

$$g_1(\theta) = L_1\cos\theta_1 + L_2\cos(\theta_1+\theta_2) + L_3\cos(\theta_1+\theta_2+\theta_3) + L_4\cos(\theta_1+\theta_2+\theta_3+\theta_4) = 0$$
$$g_2(\theta) = L_1\sin\theta_1 + L_2\sin(\theta_1+\theta_2) + \cdots = 0$$
$$g_3(\theta) = \theta_1+\theta_2+\theta_3+\theta_4 - 2\pi = 0$$

Are these holonomic or nonholonomic? How many DOF does the C-space have?

<details>
<summary><b>Solution</b></summary>

**Type:** These are **HOLONOMIC** constraints — they are explicit functions of configuration $\theta$ only (no velocities appear). The velocity constraints are obtained by differentiating $g(\theta) = 0$:

$$\frac{\partial g}{\partial \theta}\,\dot{\theta} = 0$$

which is integrable by construction (it came from $g(\theta) = 0$).

**DOF of C-space:**
- $n = 4$ joint variables
- $k = 3$ independent constraints
- C-space dimension $= n - k = 4 - 3 = \boxed{1}$ ✓

(Consistent with Grübler: four-bar linkage has 1 DOF.) ✓

</details>

---

### ▶ Medium — Integrability Test on a Pfaffian Constraint

**Question:** Determine whether the following Pfaffian constraint is holonomic or nonholonomic:

$$A(q)\dot{q} = 0 \quad \text{where} \quad A(q) = [q_2, \; q_1, \; 1]$$

i.e., $q_2\dot{q}_1 + q_1\dot{q}_2 + \dot{q}_3 = 0$.

<details>
<summary><b>Solution</b></summary>

**Apply the integrability test:** check if $\partial A_i / \partial q_j = \partial A_j / \partial q_i$ for all $i, j$.

$$A_1 = q_2, \quad A_2 = q_1, \quad A_3 = 1$$

| Pair | LHS | RHS | Equal? |
|------|-----|-----|--------|
| $(1,2)$ | $\partial A_1/\partial q_2 = 1$ | $\partial A_2/\partial q_1 = 1$ | ✓ |
| $(1,3)$ | $\partial A_1/\partial q_3 = 0$ | $\partial A_3/\partial q_1 = 0$ | ✓ |
| $(2,3)$ | $\partial A_2/\partial q_3 = 0$ | $\partial A_3/\partial q_2 = 0$ | ✓ |

All cross-partial derivatives match → the constraint **IS integrable** → **HOLONOMIC**.

**Finding $g(q)$:** We need $g$ such that $\nabla g = [q_2, q_1, 1]^T$.

- From $\partial g/\partial q_1 = q_2$: $g = q_1 q_2 + h(q_2, q_3)$
- From $\partial g/\partial q_2 = q_1$: $\partial h/\partial q_2 = 0 \Rightarrow h = k(q_3)$
- From $\partial g/\partial q_3 = 1$: $k'(q_3) = 1 \Rightarrow k(q_3) = q_3 + C$

$$\boxed{g(q) = q_1 q_2 + q_3 = \text{const}} \quad \checkmark$$

</details>

---

### ▶ Hard — Rolling Coin: Nonholonomic Constraints

**Question:** An upright coin of radius $r$ rolls without slipping on a plane. Its configuration is $q = (x, y, \phi, \theta) \in \mathbb{R}^2 \times T^2$, where $(x,y)$ is the contact point, $\phi$ is the steering angle, and $\theta$ is the spin angle. (a) Write the no-slip Pfaffian constraints. (b) Show that these constraints are nonholonomic. (c) What is the dimension of the C-space vs. feasible velocities?

<details>
<summary><b>Solution</b></summary>

**(a) No-slip constraints:**

The coin rolls in direction $(\cos\phi, \sin\phi)$ with speed $r\dot{\theta}$:

$$\dot{x} = r\dot{\theta}\cos\phi \quad \Rightarrow \quad \dot{x} - r\cos\phi \cdot \dot{\theta} = 0$$
$$\dot{y} = r\dot{\theta}\sin\phi \quad \Rightarrow \quad \dot{y} - r\sin\phi \cdot \dot{\theta} = 0$$

In matrix form $A(q)\dot{q} = 0$ with $q = [x, y, \phi, \theta]^T$:

$$A(q) = \begin{bmatrix} 1 & 0 & 0 & -r\cos\phi \\ 0 & 1 & 0 & -r\sin\phi \end{bmatrix}$$

**(b) Nonholonomic proof:**

Attempt to find $g_1(q)$ such that $\partial g_1 / \partial q = [1, 0, 0, -r\cos\phi]$:

- $\partial g_1/\partial x = 1 \Rightarrow g_1 = x + h(y, \phi, \theta)$
- $\partial g_1/\partial y = 0 \Rightarrow \partial h/\partial y = 0 \Rightarrow h = k(\phi, \theta)$
- $\partial g_1/\partial \phi = 0 \Rightarrow \partial k/\partial \phi = 0 \Rightarrow k = m(\theta)$
- $\partial g_1/\partial \theta = -r\cos\phi \Rightarrow m'(\theta) = -r\cos\phi$

But $m$ depends only on $\theta$, while $-r\cos\phi$ depends on $\phi$. **No such $m(\theta)$ can satisfy this for all $\phi$.**

$\Rightarrow$ No $g_1$ exists $\Rightarrow$ constraint is **NONHOLONOMIC** ✓

**(c)**
| | Dimension |
|--|--|
| **C-space** | **4** — the coin can reach any $(x, y, \phi, \theta)$ despite the rolling constraints (by suitable steering) |
| **Feasible velocities** | **4 − 2 = 2** — the 2 Pfaffian constraints restrict instantaneous motion to a 2D subspace (roll forward/backward + steer) |

> **KEY INSIGHT:** Nonholonomic constraints reduce velocity freedom but **NOT** the reachable configuration space. ✓

</details>

---

# Section 2.5 — Task Space and Workspace {#section-25}

## Definitions

**Task space:** The space in which the robot's task can be naturally expressed. It is defined by the **task**, not the robot. For example, a writing robot's task space is $\mathbb{R}^2$ (position on paper).

**Workspace:** The set of all end-effector configurations **reachable** by at least one robot configuration. It is defined by the **robot's structure** and joint limits. Every point in the workspace is reachable; some task-space points may not be.

> **Critical distinctions:**
> - C-space $\neq$ Task space $\neq$ Workspace
> - A task-space point may be reachable by **multiple** C-space configurations (redundant robot)
> - Two robots with **different** C-spaces can have the **same** workspace
> - Two robots with the **same** C-space can have **different** workspaces

---

## Worked Examples

### ▶ Easy — Task Space for a Planar Writing Robot

**Question:** A planar 2R robot arm is used to move a pen across a flat sheet of paper. (a) What is the task space? (b) What is the C-space? (c) Are they the same?

<details>
<summary><b>Solution</b></summary>

**(a) Task space:** The task is to position the pen tip on the paper. The relevant information is the 2D Cartesian position of the pen tip: $(x, y) \in \mathbb{R}^2$.

$$\text{Task space} = \mathbb{R}^2$$

**(b) C-space:** The robot has 2 revolute joints with no limits.

$$\text{C-space} = T^2 = S^1 \times S^1 \quad \text{(a 2D torus)}$$

**(c) They are NOT the same:**
- C-space ($T^2$) has torus topology; task space ($\mathbb{R}^2$) is flat — topologically distinct.
- The forward kinematics map $(\theta_1, \theta_2) \to (x, y)$ is many-to-one for most workspace points (two "elbow" configurations can reach the same tip position).
- Some $(x, y) \in \mathbb{R}^2$ may not be reachable (outside workspace). ✓

</details>

---

### ▶ Medium — SCARA Robot: Task Space, Workspace, and C-Space

**Question:** The SCARA robot is an RRRP open chain. Its end-effector is described by $(x, y, z, \phi)$, where $(x, y, z)$ is Cartesian position and $\phi$ is the xy-plane orientation angle. (a) State the task space. (b) State the C-space (no joint limits). (c) Define a natural workspace.

<details>
<summary><b>Solution</b></summary>

**(a) Task space:**

$$\text{Task space topology} = \mathbb{R}^3 \times S^1, \quad \text{dimension} = 4$$

(three translational DOF + one rotational DOF about the z-axis)

**(b) C-space (no joint limits):**

| Joint | Type | C-space contribution |
|-------|------|---------------------|
| 1, 2, 3 | Revolute | $S^1$ each |
| 4 | Prismatic | $\mathbb{R}^1$ |

$$\text{C-space} = T^3 \times \mathbb{R}^1, \quad \text{dimension} = 4$$

**(c) Workspace:** The set of $(x, y, z)$ positions reachable by the tip, where for each $(x, y, z)$ all orientations $\phi \in S^1$ can be achieved. The workspace is a subset of $\mathbb{R}^3$ — typically an **annular region** in the horizontal plane swept through a height range determined by the prismatic joint limits. ✓

</details>

---

### ▶ Hard — Spray-Painting Robot: Redundancy Analysis

**Question:** A 6R industrial robot is adapted for spray painting. The spray nozzle is the end-effector. What matters for the task is the Cartesian position $(x, y, z)$ of the nozzle and the direction $(\theta, \psi)$ in which it points (spherical coordinates). Rotation about the spray axis is irrelevant. (a) State the task space and its topology. (b) State the C-space (no joint limits). (c) Is the robot redundant for this task? Explain the implications.

<details>
<summary><b>Solution</b></summary>

**(a) Task space:**

| Component | Space | Reason |
|-----------|-------|--------|
| Nozzle position $(x, y, z)$ | $\mathbb{R}^3$ | Cartesian position |
| Spray direction $(\theta, \psi)$ | $S^2$ | Unit sphere of directions |

$$\text{Task space} = \mathbb{R}^3 \times S^2, \quad \text{dimension} = 5$$

(Rotation about the spray axis is irrelevant to the task.)

**(b) C-space:**

6 revolute joints with no limits, each in $S^1$:

$$\text{C-space} = T^6, \quad \text{dimension} = 6$$

**(c) Redundancy analysis:**

$$\text{Robot DOF} = 6 > \text{Task DOF} = 5 \quad \Rightarrow \quad \text{Robot is REDUNDANT}$$

**Implications:**
- For any reachable task configuration, there is (generically) a **1-dimensional family** of robot configurations (a curve in C-space) that achieve it.
- This redundancy can be exploited to:
  - Avoid obstacles in the workspace
  - Avoid kinematic singularities
  - Optimize secondary objectives (e.g., minimize joint torques or energy) while maintaining the task
- The Jacobian mapping C-space velocities to task-space velocities is a $5 \times 6$ matrix with a **1-dimensional null space** at non-singular configurations. ✓

</details>

---

# Quick-Reference Summary Table {#summary}

| Concept | Key Formula / Fact |
|---------|-------------------|
| DOF of planar rigid body | 3 DOF (x, y, θ) |
| DOF of spatial rigid body | 6 DOF (x, y, z, α, β, γ) |
| General DOF rule | DOF = variables − independent constraints |
| Grübler's (planar, $m=3$) | $\text{dof} = 3(N-1-J) + \sum f_i$ |
| Grübler's (spatial, $m=6$) | $\text{dof} = 6(N-1-J) + \sum f_i$ |
| Revolute joint | $f=1$; planar $c=2$; spatial $c=5$ |
| Prismatic joint | $f=1$; planar $c=2$; spatial $c=5$ |
| Universal joint | $f=2$; spatial $c=4$ |
| Spherical joint | $f=3$; spatial $c=3$ |
| Holonomic constraint | $g(\theta)=0$; reduces C-space dim; $A(\theta)\dot\theta=0$ is integrable |
| Nonholonomic constraint | $A(\theta)\dot\theta=0$ non-integrable; reduces velocity dim only |
| C-space of $n$R robot (no limits) | $T^n = S^1 \times S^1 \times \cdots \times S^1$ |
| Spatial rigid body C-space topology | $\mathbb{R}^3 \times S^2 \times S^1$ |
| SO(3) implicit representation | 9 variables − 6 constraints = 3 rotational DOF |
| Task space | Space where task is expressed; defined by **task** |
| Workspace | Set of reachable end-effector configs; defined by **robot** |

---

*Modern Robotics, Lynch & Park (2017) — Chapter 2 Exam Prep Notes*