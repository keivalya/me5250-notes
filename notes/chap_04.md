# Chapter 4: Forward Kinematics — Exam Preparation Notes

## Table of Contents
1. [Overview of Forward Kinematics](#1-overview)
2. [Identifying Screw Axes from Robot Geometry](#2-identifying-screw-axes)
3. [Product of Exponentials — Space Form](#3-poe-space-form)
4. [Computing Matrix Exponentials for Joints](#4-computing-matrix-exponentials)
5. [Product of Exponentials — Body Form](#5-poe-body-form)
6. [Full Forward Kinematics Computation](#6-full-fk-computation)
7. [Sketching Robots from Screw Axes and M](#7-sketching-robots)
8. [Denavit–Hartenberg Parameters](#8-dh-parameters)

---

## 1. Overview

**Forward kinematics**: Given joint values θ = [θ₁, ..., θₙ], compute end-effector configuration T ∈ SE(3).

Two main PoE representations:

| Method | Formula | What you define |
|--------|---------|-----------------|
| Space form | T(θ) = e^{[S₁]θ₁} ··· e^{[Sₙ]θₙ} M | M, S₁...Sₙ in {s} |
| Body form | T(θ) = M e^{[B₁]θ₁} ··· e^{[Bₙ]θₙ} | M, B₁...Bₙ in {b} |

**Why this matters on exams**: Nearly every exam has a problem requiring you to (a) identify screw axes from a figure, (b) compute forward kinematics for given joint values, (c) sketch the robot, and/or (d) find D-H parameters.

---

## 2. Identifying Screw Axes from Robot Geometry

### Concept

Given a robot at its **home (zero) position**, identify each joint's screw axis S_i = [ω_i; v_i] in the space frame.

**Revolute joint** (zero pitch, h = 0):
```
ω_i = unit vector along joint axis (RHR for positive rotation)
v_i = -ω_i × q_i       (equivalently, v_i = q_i × ω_i)
```
where q_i is ANY point on the joint axis in {s} coordinates.

**Physical interpretation of v_i**: Imagine joint i spinning at 1 rad/s with all other joints frozen at zero. v_i is the velocity of the point on the rigid body currently at the {s} origin.

**Prismatic joint** (infinite pitch):
```
ω_i = [0, 0, 0]^T
v_i = unit vector in direction of positive translation
```

**Quick sanity checks**:
- Revolute: ‖ω_i‖ = 1, and v_i · ω_i = 0 (always, for zero-pitch screws)
- Prismatic: ω_i = 0, ‖v_i‖ = 1
- If revolute axis passes through {s} origin: v_i = 0
- ‖v_i‖ = distance from joint axis to {s} origin (for revolute joints)

### Practice Problems

**Easy**: A 2R planar robot has joints at the {s} origin and at [L₁, 0, 0], both rotating about ẑ. The end-effector is at [L₁ + L₂, 0, 0] with R = I. Find S₁, S₂, and M.

**Solution**:

Both joints revolute about ẑ: ω₁ = ω₂ = [0, 0, 1]^T.

Joint 1 at origin: v₁ = -ω₁ × [0,0,0] = [0, 0, 0]

Joint 2 at q₂ = [L₁, 0, 0]: v₂ = -[0,0,1] × [L₁, 0, 0] = -[0, L₁, 0] = [0, -L₁, 0]

```
M = | 1  0  0  L₁+L₂ |
    | 0  1  0    0    |
    | 0  0  1    0    |
    | 0  0  0    1    |
```

| i | ω_i | v_i |
|---|-----|-----|
| 1 | (0, 0, 1) | (0, 0, 0) |
| 2 | (0, 0, 1) | (0, −L₁, 0) |

---

**Medium**: For the 3R spatial open chain (textbook Figure 4.3), joint 1 rotates about ẑ₀ through origin, joint 2 rotates about ŷ₀ through [L₁, 0, 0], and joint 3 rotates about x̂₀ through [0, 0, L₂] (with the arm in home position). The end-effector at home is:

```
M = | 0  0  -1   L₁ |
    | 0  1   0    0 |
    | 1  0   0   L₂ |
    | 0  0   0    1 |
```

Find S₁, S₂, S₃.

**Solution**:

Joint 1: ω₁ = [0, 0, 1], q₁ = [0, 0, 0]
v₁ = -[0,0,1] × [0,0,0] = [0, 0, 0]

Joint 2: ω₂ = [0, 1, 0], q₂ = [L₁, 0, 0]
ω₂ × q₂ = [0,1,0] × [L₁, 0, 0] = [(1)(0)-(0)(0), (0)(L₁)-(0)(0), (0)(0)-(1)(L₁)] = [0, 0, -L₁]
v₂ = -[0, 0, -L₁] = [0, 0, L₁]

Joint 3: ω₃ = [1, 0, 0], q₃ = [0, 0, L₂]
ω₃ × q₃ = [1,0,0] × [0, 0, L₂] = [(0)(L₂)-(0)(0), (0)(0)-(1)(L₂), (1)(0)-(0)(0)] = [0, -L₂, 0]
v₃ = -[0, -L₂, 0] = [0, L₂, 0]

| i | ω_i | v_i |
|---|-----|-----|
| 1 | (0, 0, 1) | (0, 0, 0) |
| 2 | (0, 1, 0) | (0, 0, L₁) |
| 3 | (1, 0, 0) | (0, L₂, 0) |

Matches textbook Example 4.1. ✓

---

**Hard**: A 3-DOF robot (RRP) has:
- Joint 1: revolute about ẑ_s, axis through origin
- Joint 2: revolute about ẑ_s, axis through [L, -L, 0]
- Joint 3: prismatic along -ŷ_s direction

The end-effector at home position is at [2L, -L, 0] with orientation same as {s}, but with body-frame offset d below.

```
M = | 1  0  0   2L |
    | 0  1  0   -L |
    | 0  0  1    0 |
    | 0  0  0    1 |
```

Find all screw axes.

**Solution**:

Joint 1 (revolute, ẑ through origin):
ω₁ = [0, 0, 1], q₁ = [0, 0, 0]
v₁ = [0, 0, 0]

Joint 2 (revolute, ẑ through [L, -L, 0]):
ω₂ = [0, 0, 1], q₂ = [L, -L, 0]
ω₂ × q₂ = [0,0,1] × [L, -L, 0] = [(0)(0)-(1)(-L), (1)(L)-(0)(0), (0)(-L)-(0)(L)] = [L, L, 0]
v₂ = -[L, L, 0] = [-L, -L, 0]

Check: ‖v₂‖ = L√2 = distance from [L,-L,0] to origin. ✓

Joint 3 (prismatic along -ŷ):
ω₃ = [0, 0, 0], v₃ = [0, -1, 0]

| i | ω_i | v_i |
|---|-----|-----|
| 1 | (0, 0, 1) | (0, 0, 0) |
| 2 | (0, 0, 1) | (−L, −L, 0) |
| 3 | (0, 0, 0) | (0, −1, 0) |

---

## 3. Product of Exponentials — Space Form

### Concept

The **space form** of the PoE formula:

```
T(θ) = e^{[S₁]θ₁} · e^{[S₂]θ₂} · ··· · e^{[Sₙ]θₙ} · M
```

**Key idea**: Each joint applies a screw motion to ALL outward links. Joint n moves only the end-effector. Joint n-1 moves links n-1 and n together. Joint 1 moves the entire robot.

**Why the screw axes don't change**: S_i is defined in the fixed frame. Even though distal joints move, the *space-frame representation* of a proximal joint's screw axis is unaffected by distal joint motions.

**Order of operations** (right to left): M is first transformed by the most distal joint (n), then progressively by more proximal joints toward joint 1.

### Practice Problems

**Easy**: For a 1R robot with S₁ = [0, 0, 1, 0, 0, 0]^T and M = [I, [L, 0, 0]^T; 0, 1], find T(θ₁ = π/2).

**Solution**:

```
e^{[S₁]π/2} = Rot(ẑ, π/2) with no translation (axis through origin)

= | 0  -1  0  0 |
  | 1   0  0  0 |
  | 0   0  1  0 |
  | 0   0  0  1 |

T = e^{[S₁]π/2} · M = | 0  -1  0  0 || 1  0  0  L |   | 0  -1  0  0 |
                        | 1   0  0  0 || 0  1  0  0 | = | 1   0  0  L |
                        | 0   0  1  0 || 0  0  1  0 |   | 0   0  1  0 |
                        | 0   0  0  1 || 0  0  0  1 |   | 0   0  0  1 |
```

The end-effector moved from [L, 0, 0] to [0, L, 0] and rotated 90° — the link swept a quarter circle. ✓

---

**Medium**: For a 2R planar robot with:
```
S₁ = [0, 0, 1, 0, 0, 0]^T
S₂ = [0, 0, 1, 0, -1, 0]^T    (joint at [1, 0, 0], L₁ = 1)

M = | 1  0  0  2 |
    | 0  1  0  0 |     (L₁ = L₂ = 1)
    | 0  0  1  0 |
    | 0  0  0  1 |
```

Find T for θ = [π/2, -π/2].

**Solution**:

Step 1: e^{[S₂](-π/2)} — revolute about ẑ through [1, 0, 0] by -π/2.

Using e^{[S]θ} for a revolute joint (‖ω‖ = 1):

```
Rotation part: e^{[ω]θ} = Rot(ẑ, -π/2) = | 0   1  0 |
                                             |-1   0  0 |
                                             | 0   0  1 |

Translation part: G(θ)v, where v = [0, -1, 0], θ = -π/2

G(θ) = Iθ + (1-cosθ)[ω] + (θ - sinθ)[ω]²

θ = -π/2, sinθ = -1, cosθ = 0

[ω] = | 0  -1  0 |     [ω]² = |-1  0  0 |
      | 1   0  0 |            | 0 -1  0 |
      | 0   0  0 |            | 0  0  0 |

G(-π/2) = (-π/2)I + (1)| 0  -1  0| + (-π/2+1)|-1  0  0|
                         | 1   0  0|            | 0 -1  0|
                         | 0   0  0|            | 0  0  0|

= |-π/2+π/2-1   -1        0  |   |-1  -1    0  |
  | 1         -π/2+π/2-1  0  | = | 1  -1    0  |
  | 0            0       -π/2|   | 0   0  -π/2 |

G(-π/2)v = |-1  -1    0 || 0|   | 1|
           | 1  -1    0 ||-1| = | 1|
           | 0   0  -π/2|| 0|   | 0|

e^{[S₂](-π/2)} = | 0   1  0  1 |
                   |-1   0  0  1 |
                   | 0   0  1  0 |
                   | 0   0  0  1 |
```

Step 2: e^{[S₁](π/2)} — revolute about ẑ through origin by π/2.
```
e^{[S₁](π/2)} = | 0  -1  0  0 |
                  | 1   0  0  0 |
                  | 0   0  1  0 |
                  | 0   0  0  1 |
```

Step 3: T = e^{[S₁]θ₁} · e^{[S₂]θ₂} · M
```
First: e^{[S₂](-π/2)} · M = | 0   1  0  1 || 1  0  0  2 |   | 0  1  0  1 |
                              |-1   0  0  1 || 0  1  0  0 | = |-1  0  0 -1 |
                              | 0   0  1  0 || 0  0  1  0 |   | 0  0  1  0 |
                              | 0   0  0  1 || 0  0  0  1 |   | 0  0  0  1 |

Then: T = e^{[S₁](π/2)} · (above) = | 0  -1  0  0 || 0  1  0  1 |
                                       | 1   0  0  0 ||-1  0  0 -1 |
                                       | 0   0  1  0 || 0  0  1  0 |
                                       | 0   0  0  1 || 0  0  0  1 |

    = | 1  0  0  1 |
      | 0  1  0  1 |
      | 0  0  1  0 |
      | 0  0  0  1 |
```

**End-effector at [1, 1, 0], orientation = I.** The arm bends into an "L" shape.

---

**Hard**: (Exam-level, based on Practice Exam Problem 3)

Given a 3-DOF robot with:
```
M = | 1  0  0   2L |        S₁ = [0,0,1,0,0,0]^T
    | 0  1  0   -L |        S₂ = [0,0,0,0,-1,0]^T
    | 0  0  1    0 |        S₃ = [0,0,1,-L,-L,0]^T
    | 0  0  0    1 |
```

Find T_sb for θ = [0, L, -π/2].

**Solution**:

Step 1: Identify joint types from screw axes.
- S₁: ω₁ = [0,0,1] → revolute about ẑ through origin
- S₂: ω₂ = [0,0,0], v₂ = [0,-1,0] → prismatic along -ŷ
- S₃: ω₃ = [0,0,1], v₃ = [-L,-L,0] → revolute about ẑ through point q₃

For S₃, find q₃: v₃ = -ω₃ × q₃ → [-L,-L,0] = -[0,0,1] × [qx, qy, 0] = -[−qy, qx, 0] = [qy, -qx, 0]
So qy = -L, -qx = -L → qx = L. Thus q₃ = [L, -L, 0].

Step 2: Compute each matrix exponential.

**e^{[S₁]·0} = I** (θ₁ = 0)

**e^{[S₂]·L}** (prismatic, ω = 0):
```
e^{[S₂]L} = | 1  0  0   0 |
             | 0  1  0  -L |
             | 0  0  1   0 |
             | 0  0  0   1 |
```

**e^{[S₃]·(-π/2)}** (revolute about ẑ through [L, -L, 0]):

Rotation: Rot(ẑ, -π/2) = | 0  1  0 |
                           |-1  0  0 |
                           | 0  0  1 |

Translation G(θ)v with v = [-L, -L, 0], θ = -π/2, sinθ = -1, cosθ = 0:

```
G(-π/2) = (-π/2)I + (1-0)[ω] + (-π/2-(-1))[ω]²

= (-π/2)I + |0 -1 0| + (1-π/2)|-1 0 0|
              |1  0 0|           |0 -1 0|
              |0  0 0|           |0  0 0|

= |-π/2-1+π/2    -1        0 |   |-1   -1    0  |
  |  1       -π/2-1+π/2    0 | = | 1   -1    0  |
  |  0          0        -π/2|   | 0    0  -π/2 |

G(θ)v = |-1  -1   0 ||-L|   | L+L |   | 2L|
        | 1  -1   0 ||-L| = |-L+L | = |  0|
        | 0   0  -π/2|| 0|   |  0  |   |  0|
```

```
e^{[S₃](-π/2)} = | 0   1  0  2L |
                   |-1   0  0   0 |
                   | 0   0  1   0 |
                   | 0   0  0   1 |
```

Step 3: Multiply T = I · e^{[S₂]L} · e^{[S₃](-π/2)} · M

```
e^{[S₂]L} · e^{[S₃](-π/2)} = | 1  0  0   0 || 0   1  0  2L |   | 0   1  0   2L|
                                | 0  1  0  -L ||-1   0  0   0 | = |-1   0  0   -L|
                                | 0  0  1   0 || 0   0  1   0 |   | 0   0  1    0|
                                | 0  0  0   1 || 0   0  0   1 |   | 0   0  0    1|

× M = | 0   1  0   2L|| 1  0  0  2L |   | 0  1  0  2L-L |   | 0  1  0    L|
      |-1   0  0   -L|| 0  1  0  -L | = |-1  0  0 -L-2L| = |-1  0  0  -3L|
      | 0   0  1    0|| 0  0  1   0 |   | 0  0  1    0  |   | 0  0  1    0|
      | 0   0  0    1|| 0  0  0   1 |   | 0  0  0    1  |   | 0  0  0    1|
```

**Result**:
```
T_sb = | 0   1  0    L |
       |-1   0  0  -3L |
       | 0   0  1    0 |
       | 0   0  0    1 |
```

Position: [L, -3L, 0]^T. Orientation: 90° CW rotation about ẑ. ✓

---

## 4. Computing Matrix Exponentials for Joints

### Concept

This is the core computation skill. Given [S_i] and θ_i, compute e^{[S_i]θ_i}.

**Case 1: Revolute joint** (‖ω‖ = 1)

```
e^{[S]θ} = | e^{[ω]θ}   G(θ)v |
           |    0          1   |
```

where:
```
e^{[ω]θ} = I + sinθ [ω] + (1 - cosθ)[ω]²        (Rodrigues')
G(θ) = Iθ + (1 - cosθ)[ω] + (θ - sinθ)[ω]²
```

**Case 2: Prismatic joint** (ω = 0, ‖v‖ = 1)

```
e^{[S]θ} = | I   vθ |
           | 0    1 |
```

**Special shortcut**: If a revolute axis passes through the {s} origin (v = 0):
```
e^{[S]θ} = | e^{[ω]θ}   0 |
           |    0        1 |
```
This is just a pure rotation with no translation — much simpler!

**Common rotation matrices you should memorize**:

```
Rot(ẑ, 90°) = | 0  -1  0 |    Rot(ẑ, -90°) = | 0   1  0 |
               | 1   0  0 |                     |-1   0  0 |
               | 0   0  1 |                     | 0   0  1 |

Rot(x̂, 90°) = | 1   0   0 |   Rot(ŷ, 90°) = | 0  0  1 |
               | 0   0  -1 |                   | 0  1  0 |
               | 0   1   0 |                   |-1  0  0 |
```

### Practice Problems

**Easy**: Compute e^{[S]θ} for a prismatic joint S = [0, 0, 0, 0, 0, 1]^T with θ = d.

**Solution**:
ω = 0, v = [0, 0, 1]. Translation along ẑ by d:
```
e^{[S]d} = | 1  0  0  0 |
           | 0  1  0  0 |
           | 0  0  1  d |
           | 0  0  0  1 |
```

---

**Medium**: Compute e^{[S]θ} for S = [0, 0, 1, 0, -L, 0]^T (revolute about ẑ through [L, 0, 0]) with θ = π/2.

**Solution**:

ω = [0, 0, 1], v = [0, -L, 0], θ = π/2, sinθ = 1, cosθ = 0.

```
e^{[ω]π/2} = Rot(ẑ, 90°) = | 0  -1  0 |
                              | 1   0  0 |
                              | 0   0  1 |
```

G(π/2) with [ω] and [ω]² for ẑ-axis rotation:
```
[ω] = | 0  -1  0 |     [ω]² = |-1  0  0 |
      | 1   0  0 |            | 0 -1  0 |
      | 0   0  0 |            | 0  0  0 |

G(π/2) = (π/2)I + (1)| 0 -1 0| + (π/2 - 1)|-1 0 0|
                       | 1  0 0|              | 0 -1 0|
                       | 0  0 0|              | 0  0 0|

= | π/2-π/2+1    -1        0   |   | 1   -1    0  |
  |  1        π/2-π/2+1    0   | = | 1    1    0  |
  |  0           0        π/2  |   | 0    0   π/2 |

G(π/2)v = | 1  -1   0 || 0|   |  L |
          | 1   1   0 ||-L| = | -L |
          | 0   0  π/2|| 0|   |  0 |
```

```
e^{[S]π/2} = | 0  -1  0   L |
              | 1   0  0  -L |
              | 0   0  1   0 |
              | 0   0  0   1 |
```

**Verification**: A point on the rotation axis must remain fixed. Using [L,0,0] (which is on the axis):
e^{[S]π/2} [L,0,0,1]^T = [0·L+(-1)·0+0·0+L, 1·L+0·0+0·0+(-L), 0+0+1·0+0, 1] = [L, 0, 0, 1]. ✓

---

**Hard**: Compute e^{[S]θ} for S = [0, 1, 0, -H, 0, 0]^T (revolute about ŷ through [0, 0, H]) with θ = -π/2.

**Solution**:

ω = [0, 1, 0], v = [-H, 0, 0], θ = -π/2, sinθ = -1, cosθ = 0.

```
[ω] = | 0   0   1 |     [ω]² = |-1  0  0 |
      | 0   0   0 |            | 0  0  0 |
      |-1   0   0 |            | 0  0 -1 |

e^{[ω](-π/2)} = I + (-1)| 0  0  1| + (1)|-1  0  0|
                          | 0  0  0|      | 0  0  0|
                          |-1  0  0|      | 0  0 -1|

= | 1-1    0    -1 |   | 0   0  -1 |
  |  0     1     0 | = | 0   1   0 |
  |  1     0    1-1|   | 1   0   0 |

G(-π/2) = (-π/2)I + (1)[ω] + (-π/2+1)[ω]²

= | -π/2+π/2-1    0       1    |   | -1    0    1  |
  |  0          -π/2      0    | = |  0  -π/2   0  |
  | -1            0   -π/2+π/2-1|  | -1    0   -1  |

G(-π/2)v = | -1    0    1 ||-H|   |  H  |
           |  0  -π/2   0 || 0 | = |  0  |
           | -1    0   -1 || 0 |   |  H  |
```

```
e^{[S](-π/2)} = | 0   0  -1   H |
                  | 0   1   0   0 |
                  | 1   0   0   H |
                  | 0   0   0   1 |
```

**Verify**: Point on axis [0, 0, H] should be fixed:
[0·0+0·0+(-1)H+H, 0+0+0+0, 1·0+0+0·H+H, 1] = [0, 0, H, 1]. ✓

---

## 5. Product of Exponentials — Body Form

### Concept

The **body form** of the PoE:

```
T(θ) = M · e^{[B₁]θ₁} · e^{[B₂]θ₂} · ··· · e^{[Bₙ]θₙ}
```

Each B_i is the screw axis of joint i expressed in the **end-effector frame {b}** when robot is at home.

**Relationship to space form**:
```
B_i = [Ad_{M⁻¹}] S_i
```

**Key insight**: In the body form, M is first transformed by joint 1, then joint 2, ..., then joint n. The body-frame representation of a distal joint's screw axis is NOT affected by proximal joint displacements.

**When to use which form**:
- Space form: natural when screw axes are easy to read from a figure (most common on exams)
- Body form: useful when joint axes are easier to express relative to the tool frame

### Practice Problems

**Easy**: For a 1R robot with S₁ = [0, 0, 1, 0, 0, 0]^T and M = [I, [L, 0, 0]; 0, 1], find B₁.

**Solution**:

```
M⁻¹ = | 1  0  0  -L |
       | 0  1  0   0 |
       | 0  0  1   0 |
       | 0  0  0   1 |

B₁ = [Ad_{M⁻¹}] S₁
```

For M⁻¹ = (I, [-L, 0, 0]):
```
[Ad_{M⁻¹}] = | R      0   |   = | I       0  |
              | [p]R   R   |     | [-L,0,0] I |

[p] = |  0   0   0 |
      |  0   0   L |
      |  0  -L   0 |

B₁ = | I   0 || 0 |   | 0 |
     |[p]  I || 0 | = | 0 |
              | 1 |   | 1 |
              | 0 |   | 0+0 |
              | 0 |   | L   |
              | 0 |   | 0   |
```

Wait, let me compute [p]ω more carefully:
```
[p] = | 0    0    0 |      [p]ω = | 0    0    0 || 0 |   | 0 |
      | 0    0    L |             | 0    0    L || 0 | = | L |
      | 0   -L    0 |             | 0   -L    0 || 1 |   | 0 |
```

v_b = [p]ω + v_s = [0, L, 0] + [0, 0, 0] = [0, L, 0]

**B₁ = [0, 0, 1, 0, L, 0]^T**

Check: The {b} frame origin is at [L, 0, 0] in {s}. From {b}'s perspective, the joint axis is at [-L, 0, 0], so with R=I:
v_b = -ω × (-L, 0, 0) = [0,0,1] × [L,0,0] = [0, L, 0]. ✓

---

**Medium**: For the 6R chain (textbook Example 4.6), verify B₁ given S₁ = [0,0,1,0,0,0] and:

```
M = | 1  0  0   0 |
    | 0  1  0  3L |
    | 0  0  1   0 |
    | 0  0  0   1 |
```

**Solution**:

```
M⁻¹ = | 1  0  0    0 |
       | 0  1  0  -3L |
       | 0  0  1    0 |
       | 0  0  0    1 |
```

p = [0, -3L, 0]:
```
[p] = | 0    0   -3L |
      | 0    0     0 |
      | 3L   0     0 |
```

B₁ = [Ad_{M⁻¹}]S₁:
ω_b = Rω_s = Iω_s = [0, 0, 1]
v_b = [p]ω_s + Iv_s = [p][0,0,1]^T + [0,0,0]

```
[p][0,0,1]^T = | 0    0   -3L || 0 |   | -3L |
               | 0    0     0 || 0 | = |  0  |
               | 3L   0     0 || 1 |   |  0  |
```

**B₁ = [0, 0, 1, -3L, 0, 0]^T**

Textbook gives B₁ = (0, 0, 1), v₁ = (-3L, 0, 0). ✓

---

**Hard**: Convert ALL space-frame screw axes to body-frame for a 3R planar robot with:

```
S₁ = [0,0,1,0,0,0]^T,  S₂ = [0,0,1,0,-L₁,0]^T,  S₃ = [0,0,1,0,-(L₁+L₂),0]^T

M = | 1  0  0  L₁+L₂+L₃ |
    | 0  1  0       0     |
    | 0  0  1       0     |
    | 0  0  0       1     |
```

**Solution**:

Let d = L₁+L₂+L₃. Then M⁻¹ = (I, [-d, 0, 0]).

```
[p_{M⁻¹}] = | 0   0   0 |
             | 0   0   d |
             | 0  -d   0 |
```

For each S_i, B_i = [Ad_{M⁻¹}]S_i. Since R = I:
ω_b = ω_s, and v_b = [p]ω_s + v_s.

[p][0,0,1]^T = [0, d, 0]:

B₁: v_b = [0, d, 0] + [0, 0, 0] = [0, d, 0]
→ **B₁ = [0, 0, 1, 0, L₁+L₂+L₃, 0]^T**

B₂: v_b = [0, d, 0] + [0, -L₁, 0] = [0, d-L₁, 0] = [0, L₂+L₃, 0]
→ **B₂ = [0, 0, 1, 0, L₂+L₃, 0]^T**

B₃: v_b = [0, d, 0] + [0, -(L₁+L₂), 0] = [0, d-L₁-L₂, 0] = [0, L₃, 0]
→ **B₃ = [0, 0, 1, 0, L₃, 0]^T**

**Physical interpretation**: B_i describes each joint axis as seen from the end-effector. Joint 3 is at distance L₃ from the end-effector, joint 2 at L₂+L₃, and joint 1 at L₁+L₂+L₃. ✓

---

## 6. Full Forward Kinematics Computation

### Concept

This section combines all previous skills into a complete exam-style problem: identify screw axes, compute exponentials, multiply, and get the final T.

**Exam strategy**:
1. Draw/identify the robot at home position
2. Write M by inspection (position and orientation of {b} in {s} at home)
3. For each joint, determine S_i = [ω_i; v_i]
4. Compute e^{[S_i]θ_i} for each joint
5. Multiply left to right: T = e^{[S₁]θ₁} ··· e^{[Sₙ]θₙ} M

**Time-saving tips**:
- If θ_i = 0, then e^{[S_i]·0} = I (skip it!)
- For prismatic joints, e^{[S]θ} is trivial (just a translation)
- For revolute joints through origin, G(θ)v = 0 (no translation component)

### Practice Problems

**Easy**: A single prismatic joint translates along ŷ. End-effector starts at [0, 2, 0] with R = I.
Find T when θ₁ = 3.

**Solution**:
S₁ = [0, 0, 0, 0, 1, 0], M = (I, [0, 2, 0]).

```
T = e^{[S₁]·3} · M = | 1  0  0  0 || 1  0  0  0 |   | 1  0  0  0 |
                       | 0  1  0  3 || 0  1  0  2 | = | 0  1  0  5 |
                       | 0  0  1  0 || 0  0  1  0 |   | 0  0  1  0 |
                       | 0  0  0  1 || 0  0  0  1 |   | 0  0  0  1 |
```

End-effector translated from [0,2,0] to [0,5,0]. ✓

---

**Medium**: (Based on exam-style) An RRPRRR robot. Only joints 2 and 5 are at non-zero values: θ₂ = -π/2, θ₅ = π/2, all others zero. Given:

```
S₂ = [0, 1, 0, -H₁, 0, 0]^T
S₅ = [0, 0, -1, -W₁, L₁+L₂, 0]^T

M = | -1   0  0   L₁+L₂    |
    |  0   0  1   W₁+W₂    |
    |  0   1  0   H₁-H₂    |
    |  0   0  0      1      |
```

(This is similar to the UR5 example.)

Find T.

**Solution**:

Since all other θ_i = 0, their exponentials are I:
T = e^{[S₂](-π/2)} · e^{[S₅](π/2)} · M

**e^{[S₂](-π/2)}**: ω = [0,1,0], v = [-H₁, 0, 0], θ = -π/2.

```
e^{[ω](-π/2)} = Rot(ŷ, -90°) = | 0  0  -1 |
                                  | 0  1   0 |
                                  | 1  0   0 |

For translation, G(-π/2)v with [ω] for ŷ-axis:

[ω] = | 0  0  1 |     [ω]² = |-1  0  0 |
      | 0  0  0 |            | 0  0  0 |
      |-1  0  0 |            | 0  0 -1 |

G(-π/2) = (-π/2)I + (1)[ω] + (1-π/2)[ω]²

= |π/2-1    0     1  |
  | 0     -π/2    0  |
  |-1       0   π/2-1|

(NOTE: for ŷ-axis, the nonzero [ω] entries affect rows 1,3)

G(-π/2)[-H₁, 0, 0]^T = [(π/2-1)(-H₁), 0, (-1)(-H₁)] = [H₁(1-π/2), 0, H₁]
```

For the full RRPRRR chain, evaluating both nonzero exponentials and multiplying by M yields the final numeric pose below (UR-style dimensions).

```
T = | 0  -1  0  0.095 |
    | 1   0  0  0.109 |
    | 0   0  1  0.988 |
    | 0   0  0    1   |
```

**Key lesson**: When multiple joints are at zero, the problem simplifies dramatically — just compute the non-zero exponentials and multiply.

---

**Hard**: (Full exam problem — based on Practice Exam Problem 3)

Given M, S₁, S₂, S₃ as in Section 3 Hard problem, with θ = [0, L, -π/2].

This was fully solved above. The answer is:
```
T_sb = | 0   1  0    L |
       |-1   0  0  -3L |
       | 0   0  1    0 |
       | 0   0  0    1 |
```

Position of {b}: [L, -3L, 0].
Orientation: x̂_b = [0,-1,0], ŷ_b = [1,0,0], ẑ_b = [0,0,1].

---

## 7. Sketching Robots from Screw Axes and M

### Concept

Given screw axes and M (but no figure), **reconstruct the robot geometry**. This is a common exam task.

**From screw axes, determine**:
1. **Joint type**: ω ≠ 0 → revolute; ω = 0 → prismatic
2. **Joint axis direction**: ω for revolute, v for prismatic
3. **Joint location** (revolute): solve v = -ω × q for q

**Finding joint location q from S_i**: For revolute joints with v ≠ 0:
- q lives on the joint axis. One way: q = (ω × v) / ‖ω‖² (gives the point on the axis closest to the {s} origin)
- Alternative: solve ω × (-q) = -v component by component

**From M, determine**:
- Columns of R: directions of {b} axes at home
- p: position of {b} origin at home

### Practice Problems

**Easy**: Given S = [0, 0, 1, 0, 0, 0]^T, what type of joint is this and where is it?

**Solution**:
ω = [0, 0, 1] ≠ 0 → **revolute** joint about ẑ.
v = [0, 0, 0] → axis passes through {s} origin.

---

**Medium**: Given S = [0, 0, 1, 0, -L, 0]^T, find the joint axis location.

**Solution**:
ω = [0, 0, 1], v = [0, -L, 0]. Revolute about ẑ.

Use q = (ω × v) / ‖ω‖²:
ω × v = [0,0,1] × [0,-L,0] = [(0)(0)-(1)(-L), (1)(0)-(0)(0), (0)(-L)-(0)(0)] = [L, 0, 0]
q = [L, 0, 0] / 1 = [L, 0, 0]

**Joint axis passes through [L, 0, 0], pointing in ẑ direction.** ✓

---

**Hard**: Given the following screw axes, sketch the robot and identify all joints.

```
S₁ = [0, 0, 1, 0, 0, 0]^T
S₂ = [0, 0, 0, 0, -1, 0]^T
S₃ = [0, 0, 1, -L, -L, 0]^T

M = | 1  0  0   2L |
    | 0  1  0   -L |
    | 0  0  1    0 |
    | 0  0  0    1 |
```

**Solution**:

**S₁**: ω₁ = [0,0,1], v₁ = [0,0,0]. Revolute about ẑ through origin.

**S₂**: ω₂ = [0,0,0], v₂ = [0,-1,0]. Prismatic joint translating in -ŷ direction.

**S₃**: ω₃ = [0,0,1], v₃ = [-L,-L,0]. Revolute about ẑ.
From v = -ω × q:
[-L, -L, 0] = [q_y, -q_x, 0]  =>  q_x = L, q_y = -L
So q₃ = [L, -L, 0].

Revolute about ẑ through [L, -L, 0].

**M**: End-effector at [2L, -L, 0], orientation aligned with {s}.

**Sketch description**: 
- Joint 1 at origin: revolute (rotates about ẑ)
- Between J1 and J2: link extends to some intermediate point
- Joint 2: prismatic, translates in -ŷ direction (connects J1 and J3 regions)
- Joint 3 at [L, -L, 0]: revolute about ẑ
- From J3 to end-effector at [2L, -L, 0]: final link of length L in x̂ direction

The robot is planar (all ẑ-axis rotations and ŷ-translation) operating in the x-y plane.

---

## 8. Denavit–Hartenberg Parameters

### Concept

The D-H convention parameterizes each link transformation with **4 parameters**: α_{i-1}, a_{i-1}, d_i, φ_i.

The transformation from frame {i-1} to frame {i}:
```
T_{i-1,i} = Rot(x̂, α_{i-1}) · Trans(x̂, a_{i-1}) · Trans(ẑ, d_i) · Rot(ẑ, φ_i)
```

**D-H frame assignment rules**:
1. ẑ_i axis is along joint i+1 axis
2. x̂_i is along the **common normal** from ẑ_{i-1} to ẑ_i
3. Origin of frame {i} is at the intersection of x̂_i with ẑ_i

**Parameter definitions**:
| Parameter | Definition |
|-----------|-----------|
| α_{i-1} | Angle from ẑ_{i-1} to ẑ_i about x̂_{i-1} (twist angle) |
| a_{i-1} | Distance from ẑ_{i-1} to ẑ_i along x̂_{i-1} (link length) |
| d_i | Distance from x̂_{i-1} to x̂_i along ẑ_i (link offset) |
| φ_i | Angle from x̂_{i-1} to x̂_i about ẑ_i (joint angle for revolute) |

**For revolute joints**: φ_i = θ_i (variable), d_i is constant
**For prismatic joints**: d_i = θ_i (variable), φ_i is constant

**Relationship to PoE**: You also need T_{s,0} and T_{n,b} to connect D-H frames to {s} and {b}:
```
T_sb = T_{s,0} · T_{0,1}(θ₁) · T_{1,2}(θ₂) · ··· · T_{n-1,n}(θₙ) · T_{n,b}
```

### Practice Problems

**Easy**: A single revolute joint about ẑ with the next joint also about ẑ, offset by L in x̂. Find D-H parameters.

**Solution**:
Both ẑ axes are parallel (both along ẑ). The common normal is along x̂.

α₀ = 0 (ẑ₀ and ẑ₁ are parallel → no twist)
a₀ = L (distance between axes along x̂)
d₁ = 0 (no offset along ẑ₁)
φ₁ = θ₁ (revolute joint variable)

| i | α_{i-1} | a_{i-1} | d_i | φ_i |
|---|---------|---------|-----|-----|
| 1 | 0 | L | 0 | θ₁ |

---

**Medium**: For the 3R planar robot (L₁, L₂, L₃), find D-H parameters.

**Solution**:
All joints rotate about ẑ (parallel axes). Common normals along x̂. All in the same plane.

| i | α_{i-1} | a_{i-1} | d_i | φ_i |
|---|---------|---------|-----|-----|
| 1 | 0 | 0 | 0 | θ₁ |
| 2 | 0 | L₁ | 0 | θ₂ |
| 3 | 0 | L₂ | 0 | θ₃ |

T_{3,b} accounts for the final link length L₃:
```
T_{3,b} = | 1  0  0  L₃ |
           | 0  1  0   0 |
           | 0  0  1   0 |
           | 0  0  0   1 |
```

---

**Hard**: (Based on Practice Exam Problem 3c) For the RRP robot with joints:
- J1: revolute about ẑ at origin
- J2: prismatic along -ŷ
- J3: revolute about ẑ through [L, -L, 0]

Find D-H parameters. (The D-H frame assignment requires care.)

**Solution**:

This requires setting up D-H frames:
- ẑ₀ along J1 axis (ẑ_s)
- ẑ₁ along J2 axis. Since J2 is prismatic along -ŷ, choose ẑ₁ = -ŷ_s.

Equivalent convention: choose ẑ₁ = +ŷ_s and carry the sign in d₂. Both are valid if used consistently.

The common normal from ẑ₀ (= ẑ_s) to ẑ₁ (= -ŷ_s): These are perpendicular, so α₁ = π/2 (or -π/2 depending on direction convention).

This gets into the specifics of D-H frame placement which can have multiple valid solutions. The key exam insight: for the D-H table, you need to carefully:
1. Assign ẑ_i along each joint axis
2. Find common normals for x̂_i
3. Read off the 4 parameters

One valid D-H assignment (from the Practice Exam solution):

| i | α_{i-1} | a_{i-1} | d_i | φ_i |
|---|---------|---------|-----|-----|
| 1 | 0 | 0 | 0 | θ₁ |
| 2 | π/2 | ℓ₁ | L + θ₂ | 0 |
| 3 | -π/2 | L - ℓ₁ | 0 | θ₃ |

where ℓ₁ is a free parameter representing the x-distance from J1 to the D-H origin for frame 1 (since the axes intersect in a somewhat non-standard way).

**Important exam note**: D-H parameters are NOT unique — different valid frame assignments give different parameters but the same overall forward kinematics. Multiple solutions can receive full credit as long as they're self-consistent.

---

## Key Formulas Quick Reference

| Concept | Formula |
|---------|---------|
| Space form PoE | T(θ) = e^{[S₁]θ₁} ··· e^{[Sₙ]θₙ} M |
| Body form PoE | T(θ) = M e^{[B₁]θ₁} ··· e^{[Bₙ]θₙ} |
| Space ↔ Body | B_i = [Ad_{M⁻¹}] S_i |
| Revolute screw axis | S = [ω̂;  -ω̂ × q] |
| Prismatic screw axis | S = [0;  v̂] |
| Revolute e^{[S]θ} | [e^{[ω]θ},  G(θ)v;  0,  1] |
| Prismatic e^{[S]θ} | [I,  vθ;  0,  1] |
| G(θ) | Iθ + (1-cosθ)[ω] + (θ-sinθ)[ω]² |
| Rodrigues' formula | e^{[ω̂]θ} = I + sinθ[ω̂] + (1-cosθ)[ω̂]² |
| Joint through origin | v = 0, so e^{[S]θ} = [Rot, 0; 0, 1] |
| θ_i = 0 shortcut | e^{[S_i]·0} = I |
| D-H transformation | Rot(x̂,α)·Trans(x̂,a)·Trans(ẑ,d)·Rot(ẑ,φ) |
| Find q from S | q = (ω × v) / ‖ω‖² |

---

## Exam Preparation Checklist

Before the exam, make sure you can:

- [ ] Given a robot figure, identify all screw axes S_i in the space frame
- [ ] Compute e^{[S]θ} for revolute joints (including G(θ)v computation)
- [ ] Compute e^{[S]θ} for prismatic joints
- [ ] Multiply 4×4 matrices correctly under time pressure
- [ ] Write M by inspection from a robot figure
- [ ] Convert between space and body form using the adjoint
- [ ] Reconstruct robot geometry from given screw axes
- [ ] Exploit θ_i = 0 to skip unnecessary computations
- [ ] Assign D-H frames and extract D-H parameters
- [ ] Verify your answer (e.g., points on joint axes should be fixed)

---

*End of Chapter 4 Exam Preparation Notes*
