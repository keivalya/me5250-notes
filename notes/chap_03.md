# Chapter 3: Rigid-Body Motions — Exam Preparation Notes

## Table of Contents
1. [Rotation Matrices and SO(3)](#1-rotation-matrices-and-so3)
2. [Properties and Uses of Rotation Matrices](#2-properties-and-uses-of-rotation-matrices)
3. [Angular Velocities](#3-angular-velocities)
4. [Exponential Coordinates for Rotations (Rodrigues' Formula)](#4-exponential-coordinates-for-rotations)
5. [Matrix Logarithm of Rotations](#5-matrix-logarithm-of-rotations)
6. [Homogeneous Transformation Matrices and SE(3)](#6-homogeneous-transformation-matrices-and-se3)
7. [Twists (Spatial Velocities)](#7-twists-spatial-velocities)
8. [Adjoint Representation](#8-adjoint-representation)
9. [Screw Axes](#9-screw-axes)
10. [Exponential Coordinates for Rigid-Body Motions](#10-exponential-coordinates-for-rigid-body-motions)
11. [Matrix Logarithm of Rigid-Body Motions](#11-matrix-logarithm-of-rigid-body-motions)
12. [Wrenches](#12-wrenches)

---

## 1. Rotation Matrices and SO(3)

### Concept

A **rotation matrix** R is a 3×3 matrix whose columns are the unit axes of a body frame {b} expressed in a fixed frame {s}:

```
R = [x̂_b  ŷ_b  ẑ_b]
```

**SO(3)** (Special Orthogonal Group) is the set of all 3×3 real matrices R satisfying:
- **Orthogonality**: R^T R = I  (columns are orthonormal)
- **Right-handedness**: det(R) = 1

These two conditions impose 6 constraints on the 9 entries, leaving **3 degrees of freedom** (matching the 3 DOF of spatial orientation).

**Key distinction**: If R^T R = I but det(R) = −1, the matrix represents a reflection, NOT a rotation.

### Practice Problems

**Easy**: Given the matrix Q below, determine if it is a valid rotation matrix.

```
Q = | 0  -1   0 |
    | 0   0   1 |
    | 1   0   0 |
```

**Solution**:
Check 1 — Orthogonality: Q^T Q:

```
Q^T = | 0   0   1 |
      |-1   0   0 |
      | 0   1   0 |

Q^T Q = | 0·0+0·0+1·1    0·(-1)+0·0+1·0    0·0+0·1+1·0 |   | 1  0  0 |
        |(-1)·0+0·0+0·1  (-1)(-1)+0·0+0·0  (-1)·0+0·1+0·0| = | 0  1  0 | = I  ✓
        | 0·0+1·0+0·1     0·(-1)+1·0+0·0    0·0+1·1+0·0  |   | 0  0  1 |
```

Check 2 — Determinant:
det(Q) = 0·(0·0 − 1·0) − (−1)·(0·0 − 1·1) + 0·(0·0 − 0·1)
       = 0 − (−1)(−1) + 0 = −1

Since det(Q) = −1 ≠ 1, **Q is NOT a rotation matrix**. It preserves vector lengths but represents a reflection.

---

**Medium**: Consider a rotation matrix R with the structure:

```
R = | a    a  |
    |-a    b  |
```

where a, b are real numbers. Find the numerical values of a and b.

**Solution**:
For R ∈ SO(2), we need R^T R = I:

```
R^T R = | a  -a || a   a | = | 2a²      a²-ab |
        | a   b || -a  b |   | a²-ab    a²+b² |
```

Setting equal to I:
- (1,1): 2a² = 1  →  a = √2/2
- (1,2): a² − ab = 0  →  a(a − b) = 0. Since a ≠ 0, b = a = √2/2
- (2,2): a² + b² = 1/2 + 1/2 = 1  ✓

Check: det(R) = ab − (−a)(a) = ab + a² = (√2/2)(√2/2) + 1/2 = 1/2 + 1/2 = 1  ✓

**Answer**: a = √2/2, b = √2/2.

---

**Hard**: Consider the matrix:

```
R = | 0  -1   0 |
    | 0   0  -1 |
    | 1   0   0 |
```

(a) Verify R ∈ SO(3) by showing R^T R = I and det(R) = 1.
(b) Show that ‖Rx‖ = ‖x‖ for arbitrary x = [x₁, x₂, x₃]^T.

**Solution**:

(a) Orthogonality check:

```
R^T = | 0   0   1 |
      |-1   0   0 |
      | 0  -1   0 |

R^T R = | 0·0+0·0+1·1     0·(-1)+0·0+1·0    0·0+0·(-1)+1·0 |   | 1  0  0 |
        |(-1)·0+0·0+0·1  (-1)(-1)+0·0+0·0   (-1)·0+0·(-1)+0·0| = | 0  1  0 | = I  ✓
        | 0·0+(-1)·0+0·1   0·(-1)+(-1)·0+0·0  0·0+(-1)(-1)+0·0|  | 0  0  1 |
```

Determinant (expanding along first column):
det(R) = 0·(0·0−(−1)·0) − 0·((−1)·0−0·0) + 1·((−1)(−1)−0·0) = 0 − 0 + 1 = 1  ✓

(b) Length preservation:

```
Rx = | 0  -1   0 || x₁ |   | -x₂ |
     | 0   0  -1 || x₂ | = | -x₃ |
     | 1   0   0 || x₃ |   |  x₁ |

‖Rx‖² = (-x₂)² + (-x₃)² + (x₁)² = x₁² + x₂² + x₃² = ‖x‖²
```

Therefore ‖Rx‖ = ‖x‖.  ✓

**Proof (general)**: For any R ∈ SO(3) and x ∈ ℝ³:
‖Rx‖² = (Rx)^T(Rx) = x^T R^T R x = x^T I x = x^T x = ‖x‖²

---

## 2. Properties and Uses of Rotation Matrices

### Concept

**Three uses of a rotation matrix R:**

1. **Represent an orientation**: R_sb represents the orientation of frame {b} in frame {s}. The columns of R_sb are the unit axes of {b} expressed in {s}.

2. **Change reference frame**: If R_ab is the orientation of {b} in {a}, and R_bc is {c} in {b}, then:
   - R_ac = R_ab · R_bc  (subscript cancellation rule)
   - p_a = R_ab · p_b  (change vector from {b} coords to {a} coords)

3. **Rotate a vector or frame**:
   - **Pre-multiply** R · R_sb = R_sb': rotation about axis ω̂ expressed in **fixed frame {s}**
   - **Post-multiply** R_sb · R = R_sb'': rotation about axis ω̂ expressed in **body frame {b}**

**Key rule**: R_de = R_ed^{-1} = R_ed^T for any two frames {d}, {e}.

### Elementary Rotation Matrices

```
Rot(x̂, θ) = | 1    0       0    |
             | 0   cosθ   -sinθ  |
             | 0   sinθ    cosθ  |

Rot(ŷ, θ) = | cosθ    0    sinθ |
             |  0      1     0   |
             |-sinθ    0    cosθ |

Rot(ẑ, θ) = | cosθ  -sinθ   0 |
             | sinθ   cosθ   0 |
             |  0      0     1 |
```

### Practice Problems

**Easy**: Frame {a} is aligned with {s}. Frame {b} is obtained by rotating {a} about ẑ_s by 90°. Find R_sb.

**Solution**:

```
R_sb = Rot(ẑ, 90°) = | cos90°  -sin90°  0 |   | 0  -1  0 |
                      | sin90°   cos90°  0 | = | 1   0  0 |
                      |   0        0     1 |   | 0   0  1 |
```

---

**Medium**: Frame {a} is initially aligned with {s}. We first rotate about the body x-axis by α, then about the **new** body z-axis by β. Find the final orientation R_sc.

**Solution**:
Since both rotations are about **body** axes, we **post-multiply**:

```
R_sc = R_sa · R_ab · R_bc = I · Rot(x̂, α) · Rot(ẑ, β)
```

```
R_sc = | 1    0       0   || cosβ  -sinβ  0 |
       | 0   cosα   -sinα || sinβ   cosβ  0 |
       | 0   sinα    cosα ||  0      0    1 |

     = |      cosβ          -sinβ          0     |
       | cosα·sinβ      cosα·cosβ       -sinα    |
       | sinα·sinβ      sinα·cosβ        cosα    |
```

For α = β = π/2:

```
R_sc = | 0   -1   0 |
       | 0    0  -1 |
       | 1    0   0 |
```

---

**Hard**: Frame {b} is related to {s} by R_sb. Show that successive body-frame rotations R₁, R₂, R₃ give the same result as fixed-frame rotations applied in **reverse** order.

**Solution**:

Body-frame rotations applied sequentially:
- Start: R_sb = I
- After body rotation R₁: R_sb' = R_sb · R₁ = R₁
- After body rotation R₂: R_sb'' = R_sb' · R₂ = R₁ · R₂
- After body rotation R₃: R_sb''' = R_sb'' · R₃ = R₁ · R₂ · R₃

Fixed-frame rotations in reverse order (R₃ first, then R₂, then R₁):
- Start: R_sb = I
- After fixed rotation R₃: R_sb' = R₃ · R_sb = R₃
- After fixed rotation R₂: R_sb'' = R₂ · R_sb' = R₂ · R₃
- After fixed rotation R₁: R_sb''' = R₁ · R_sb'' = R₁ · R₂ · R₃

Both give **R₁ · R₂ · R₃**.  ✓

This is exactly the principle behind roll–pitch–yaw: body-frame R-P-Y equals fixed-frame Y-P-R.

---

## 3. Angular Velocities

### Concept

If R(t) is a time-varying rotation matrix, the angular velocity ω can be expressed in two frames:

**Space-frame angular velocity**:
```
[ω_s] = Ṙ R^{-1} = Ṙ R^T  ∈ so(3)
```

**Body-frame angular velocity**:
```
[ω_b] = R^{-1} Ṙ = R^T Ṙ  ∈ so(3)
```

where [ω] is the **skew-symmetric matrix** representation of ω = [ω₁, ω₂, ω₃]^T:

```
[ω] = |  0    -ω₃    ω₂ |
      |  ω₃    0    -ω₁ |
      | -ω₂   ω₁     0  |
```

**Relationship**: ω_s = R · ω_b  and  ω_b = R^T · ω_s

**Important property**: R[ω]R^T = [Rω]  (Proposition 3.8)

### Practice Problems

**Easy**: Given ω = [1, 2, 3]^T, write the skew-symmetric matrix [ω].

**Solution**:
```
[ω] = |  0   -3    2 |
      |  3    0   -1 |
      | -2    1    0 |
```

Verify: [ω] = −[ω]^T  ✓

---

**Medium**: Given R_sb below and the space-frame angular velocity ω_s = [3, 2, 1]^T, find ω_b.

```
R_sb = | 0  -1  0 |
       | 1   0  0 |
       | 0   0  1 |
```

**Solution**:
ω_b = R^T ω_s:

```
ω_b = | 0   1   0 || 3 |   | 2 |
      |-1   0   0 || 2 | = |-3 |
      | 0   0   1 || 1 |   | 1 |
```

---

**Hard**: Verify that R[ω]R^T = [Rω] for R = Rot(ẑ, 90°) and ω = [1, 0, 0]^T.

**Solution**:

```
R = | 0  -1  0 |       [ω] = | 0   0   0 |
    | 1   0  0 |             | 0   0  -1 |
    | 0   0  1 |             | 0   1   0 |
```

Left side: R[ω]R^T:

```
R[ω] = | 0  -1  0 || 0   0   0 |   | 0   0   1 |
       | 1   0  0 || 0   0  -1 | = | 0   0   0 |
       | 0   0  1 || 0   1   0 |   | 0   1   0 |

R[ω]R^T = | 0   0   1 || 0   1  0 |   | 0   0   1 |
          | 0   0   0 ||-1   0  0 | = | 0   0   0 |
          | 0   1   0 || 0   0  1 |   |-1   0   0 |
```

Right side: [Rω]:

```
Rω = | 0  -1  0 || 1 |   | 0 |
     | 1   0  0 || 0 | = | 1 |
     | 0   0  1 || 0 |   | 0 |

[Rω] = | 0   0   1 |
       | 0   0   0 |
       |-1   0   0 |
```

Both sides equal.  ✓

---

## 4. Exponential Coordinates for Rotations

### Concept (Rodrigues' Formula)

Any rotation can be described by a unit axis ω̂ ∈ ℝ³ (‖ω̂‖ = 1) and an angle θ. The exponential coordinates are ω̂θ ∈ ℝ³.

**Rodrigues' formula**:
```
R = e^{[ω̂]θ} = I + sinθ [ω̂] + (1 − cosθ) [ω̂]²
```

This maps from so(3) → SO(3).

**Key identity used in derivation**: [ω̂]³ = −[ω̂]  (since ‖ω̂‖ = 1)

**Proof sketch**: Expand e^{[ω̂]θ} as a power series, use [ω̂]³ = −[ω̂] to reduce all higher powers, and recognize the Taylor series for sinθ and cosθ.

### Practice Problems

**Easy**: Compute e^{[ω̂]θ} for ω̂ = [0, 0, 1]^T and θ = π/2.

**Solution**:

```
[ω̂] = | 0  -1  0 |       [ω̂]² = | 0  -1  0 || 0  -1  0 |   |-1   0  0 |
       | 1   0  0 |               | 1   0  0 || 1   0  0 | = | 0  -1  0 |
       | 0   0  0 |               | 0   0  0 || 0   0  0 |   | 0   0  0 |
```

Apply Rodrigues':
```
R = I + sin(π/2)[ω̂] + (1 − cos(π/2))[ω̂]²

  = | 1  0  0 |     | 0  -1  0 |     |-1   0  0 |
    | 0  1  0 | + 1·| 1   0  0 | + 1·| 0  -1  0 |
    | 0  0  1 |     | 0   0  0 |     | 0   0  0 |

  = | 0  -1  0 |
    | 1   0  0 | = Rot(ẑ, π/2)  ✓
    | 0   0  1 |
```

---

**Medium**: Find the rotation matrix for ω̂ = (1/√3)[1, 1, 1]^T and θ = 2π/3 (120°).

**Solution**:

sinθ = sin(120°) = √3/2,  cosθ = cos(120°) = −1/2

```
[ω̂] = (1/√3)| 0  -1   1 |
             | 1   0  -1 |
             |-1   1   0 |
```

```
[ω̂]² = (1/3)| 0  -1   1 || 0  -1   1 |   (1/3)|-2   1   1 |
             | 1   0  -1 || 1   0  -1 | =       | 1  -2   1 |
             |-1   1   0 ||-1   1   0 |         | 1   1  -2 |
```

Apply Rodrigues':
```
R = I + (√3/2)(1/√3)| 0  -1   1 | + (3/2)(1/3)|-2   1   1 |
                     | 1   0  -1 |              | 1  -2   1 |
                     |-1   1   0 |              | 1   1  -2 |

  = I + (1/2)| 0  -1   1 | + (1/2)|-2   1   1 |
             | 1   0  -1 |        | 1  -2   1 |
             |-1   1   0 |        | 1   1  -2 |

  = | 1  0  0 |   | 0   -1/2   1/2 |   |-1    1/2   1/2 |
    | 0  1  0 | + | 1/2   0   -1/2 | + | 1/2  -1    1/2 |
    | 0  0  1 |   |-1/2  1/2    0  |   | 1/2   1/2  -1  |

  = | 0   0   1 |
    | 1   0   0 |
    | 0   1   0 |
```

**Verification**: This is a 120° rotation about the [1,1,1] axis, which cyclically permutes the coordinate axes (x→y→z→x).  ✓

---

**Hard**: Verify Rodrigues' formula by showing R R^{-1} = I using R = I + sinθ[ω̂] + (1−cosθ)[ω̂]² and R^{-1} = R(−θ) = I − sinθ[ω̂] + (1−cosθ)[ω̂]².

**Solution**:

Using R(−θ) = I − sinθ[ω̂] + (1−cosθ)[ω̂]² (since sin is odd, cos is even):

```
R · R^{-1} = (I + sinθ[ω̂] + (1−cosθ)[ω̂]²)(I − sinθ[ω̂] + (1−cosθ)[ω̂]²)
```

Expand term by term, using [ω̂]³ = −[ω̂] and [ω̂]⁴ = −[ω̂]² :

```
= I − sinθ[ω̂] + (1−cosθ)[ω̂]²
  + sinθ[ω̂] − sin²θ[ω̂]² + sinθ(1−cosθ)[ω̂]³
  + (1−cosθ)[ω̂]² − sinθ(1−cosθ)[ω̂]³ + (1−cosθ)²[ω̂]⁴
```

Simplify using [ω̂]³ = −[ω̂] and [ω̂]⁴ = −[ω̂]²:

```
= I + [ω̂]²(2(1−cosθ) − sin²θ − (1−cosθ)²)
  + [ω̂](−sinθ(1−cosθ) + sinθ(1−cosθ))  ← cancels to 0
```

The coefficient of [ω̂]²:
```
2(1−cosθ) − sin²θ − (1−cosθ)²
= 2 − 2cosθ − sin²θ − 1 + 2cosθ − cos²θ
= 1 − sin²θ − cos²θ
= 1 − 1 = 0
```

Therefore R · R^{-1} = I.  ✓

---

## 5. Matrix Logarithm of Rotations

### Concept

The matrix logarithm is the **inverse** of the matrix exponential: given R ∈ SO(3), find ω̂ and θ such that e^{[ω̂]θ} = R.

**Algorithm** (θ ∈ [0, π]):

**(a)** If R = I: θ = 0, ω̂ is undefined.

**(b)** If tr(R) = −1: θ = π, and ω̂ is found from one of:
```
ω̂ = (1/√(2(1+r₃₃))) [r₁₃, r₂₃, 1+r₃₃]^T
ω̂ = (1/√(2(1+r₂₂))) [r₁₂, 1+r₂₂, r₃₂]^T
ω̂ = (1/√(2(1+r₁₁))) [1+r₁₁, r₂₁, r₃₁]^T
```
(Use whichever avoids division by zero.)

**(c)** Otherwise: θ = cos⁻¹((tr(R)−1)/2) and:
```
[ω̂] = (1/(2sinθ))(R − R^T)
```

### Practice Problems

**Easy**: Find the axis-angle representation of R = Rot(ẑ, 60°).

```
R = | cos60°  -sin60°  0 |   | 0.5    -0.866   0 |
    | sin60°   cos60°  0 | = | 0.866   0.5     0 |
    |   0        0     1 |   | 0       0       1 |
```

**Solution**:
tr(R) = 0.5 + 0.5 + 1 = 2. Case (c) applies.

θ = cos⁻¹((2−1)/2) = cos⁻¹(0.5) = 60° = π/3  ✓

```
[ω̂] = (1/(2sin60°))(R − R^T) = (1/√3) | 0      -2·0.866    0    |
                                          | 2·0.866    0        0    |
                                          | 0          0        0    |

     = (1/√3) | 0    -1.732   0 |   | 0   -1   0 |
              | 1.732   0     0 | = | 1    0   0 |
              | 0        0    0 |   | 0    0   0 |
```

So ω̂ = [0, 0, 1]^T and θ = π/3. As expected: rotation about ẑ by 60°.

---

**Medium**: Find the axis and angle for:

```
R = | 0   -1   0 |
    | 0    0  -1 |
    | 1    0   0 |
```

**Solution**:
tr(R) = 0 + 0 + 0 = 0. Since tr(R) ≠ −1 and R ≠ I, case (c) applies.

θ = cos⁻¹((0−1)/2) = cos⁻¹(−1/2) = 2π/3 (120°)

```
R − R^T = | 0  -1  0 |   | 0   0   1 |   | 0  -1  -1 |
          | 0   0 -1 | − |-1   0   0 | = | 1   0  -1 |
          | 1   0  0 |   | 0  -1   0 |   | 1   1   0 |

[ω̂] = (1/(2sin(2π/3)))(R − R^T) = (1/√3) | 0  -1  -1 |
                                             | 1   0  -1 |
                                             | 1   1   0 |
```

Reading off the skew-symmetric entries:

```
[ω̂] = |  0    -ω̂₃    ω̂₂ |
       |  ω̂₃    0    -ω̂₁ |
       | -ω̂₂   ω̂₁     0  |
```

```
ω̂₃ = 1/√3,  ω̂₂ = −(−1/√3) = ... 
```

From [ω̂]: ω̂₁ = (1/√3)(1) = 1/√3, ω̂₂ = (1/√3)(−1) = −1/√3, ω̂₃ = (1/√3)(1) = 1/√3

```
[ω̂] = (1/√3) |  0   -1   -1 |
             |  1    0   -1 |
             |  1    1    0 |
```

So: −ω̂₃ = −1/√3 → ω̂₃ = 1/√3
    ω̂₂ = −1/√3
    −ω̂₁ = −1/√3 → ω̂₁ = 1/√3

**ω̂ = (1/√3)[1, −1, 1]^T,  θ = 2π/3**

Verify: ‖ω̂‖ = √(1/3 + 1/3 + 1/3) = 1  ✓

---

**Hard**: Find the axis-angle for R when tr(R) = −1:

```
R = |-1   0   0 |
    | 0   0   1 |
    | 0   1   0 |
```

**Solution**:
tr(R) = −1 + 0 + 0 = −1. Case (b) applies: θ = π.

Use formula (3.58): ω̂ = (1/√(2(1+r₃₃))) [r₁₃, r₂₃, 1+r₃₃]^T

r₃₃ = 0, so 1 + r₃₃ = 1:
```
ω̂ = (1/√(2·1)) [0, 1, 1]^T = (1/√2) [0, 1, 1]^T
```

Verify: ‖ω̂‖ = √(0 + 1/2 + 1/2) = 1  ✓

Verify R = I + 2[ω̂]²:
```
[ω̂] = (1/√2) | 0  -1   1 |
               | 1   0   0 |
               |-1   0   0 |

[ω̂]² = (1/2) | 0  -1   1 || 0  -1   1 |   (1/2) |-2   0   0 |   |-1   0   0 |
              | 1   0   0 || 1   0   0 | =       | 0  -1   1 | = | 0  -1/2  1/2|
              |-1   0   0 ||-1   0   0 |         | 0   1  -1 |   | 0   1/2 -1/2|

R = I + 2[ω̂]² = | 1  0  0 |   |-2   0    0 |   |-1   0   0 |
                  | 0  1  0 | + | 0  -1    1 | = | 0   0   1 |  ✓
                  | 0  0  1 |   | 0   1   -1 |   | 0   1   0 |
```

**Answer**: ω̂ = (1/√2)[0, 1, 1]^T, θ = π.

---

## 6. Homogeneous Transformation Matrices and SE(3)

### Concept

A **homogeneous transformation matrix** T ∈ SE(3) packages rotation R and translation p:

```
T = | R   p |     T^{-1} = | R^T   -R^T p |
    | 0   1 |               |  0       1   |
```

**SE(3)** requires R ∈ SO(3) and p ∈ ℝ³. The group has **6 DOF** (3 rotation + 3 translation).

**Three uses** (same pattern as rotation matrices):

1. **Represent configuration**: T_sb = configuration of {b} in {s}
2. **Change reference frame**: T_ac = T_ab · T_bc (subscript cancellation)
3. **Displace a frame**:
   - T · T_sb = T_sb': transform in **fixed frame** (first rotate about {s} axis, then translate in {s})
   - T_sb · T = T_sb'': transform in **body frame** (first translate in {b}, then rotate about {b} axis)

### Practice Problems

**Easy**: Given T_sb, find T_bs.

```
T_sb = | 1  0  0   3 |
       | 0  0 -1   2 |
       | 0  1  0   1 |
       | 0  0  0   1 |
```

**Solution**:
R = upper-left 3×3, p = [3, 2, 1]^T

```
R^T = | 1   0   0 |
      | 0   0   1 |
      | 0  -1   0 |

-R^T p = -| 1   0   0 || 3 |   | -3 |
          | 0   0   1 || 2 | = | -1 |
          | 0  -1   0 || 1 |   | -2 |

T_bs = | 1   0   0  -3 |
       | 0   0   1  -1 |
       | 0  -1   0  -2 |
       | 0   0   0   1 |
```

---

**Medium**: Given frames {s}, {b}, and {c} with:

```
T_sb = | 1  0  0  1 |        T_bc = | 0  -1  0  2 |
       | 0  1  0  2 |               | 1   0  0  0 |
       | 0  0  1  0 |               | 0   0  1  3 |
       | 0  0  0  1 |               | 0   0  0  1 |
```

Find T_sc and the position of the origin of {c} in {s}.

**Solution**:
T_sc = T_sb · T_bc:

```
T_sc = | 1  0  0  1 || 0  -1  0  2 |   | 0  -1  0  3 |
       | 0  1  0  2 || 1   0  0  0 | = | 1   0  0  2 |
       | 0  0  1  0 || 0   0  1  3 |   | 0   0  1  3 |
       | 0  0  0  1 || 0   0  0  1 |   | 0   0  0  1 |
```

Position of {c} origin in {s}: p = [3, 2, 3]^T.

---

**Hard**: A robot end-effector is at T_sb. We want to displace it by rotating 90° about ẑ_b (the body z-axis) and then translating 5 units along ŷ_b (the body y-axis).

```
T_sb = | 0  -1  0   4 |
       | 1   0  0   0 |
       | 0   0  1   2 |
       | 0   0  0   1 |
```

Find the new configuration T_sb'.

**Solution**:
Body-frame transformation → post-multiply. The displacement is Trans(p) · Rot, but since we first rotate then translate in body frame, the combined operator is:

Actually, re-reading: rotate 90° about ẑ_b, then translate 5 along ŷ_b. For body-frame operations applied sequentially:

T_sb' = T_sb · Rot(ẑ, 90°) · Trans(0, 5, 0)

But a single operator T = Trans(0,5,0) · Rot(ẑ, 90°) won't work because these are sequential body operations. Let me compute step by step:

Step 1: Rotate about ẑ_b by 90°:
```
T_sb1 = T_sb · | 0  -1  0  0 |   | 0  -1  0   4 || 0  -1  0  0 |
               | 1   0  0  0 | = | 1   0  0   0 || 1   0  0  0 |
               | 0   0  1  0 |   | 0   0  1   2 || 0   0  1  0 |
               | 0   0  0  1 |   | 0   0  0   1 || 0   0  0  1 |

       = |-1   0  0   4 |
         | 0  -1  0   0 |
         | 0   0  1   2 |
         | 0   0  0   1 |
```

Step 2: Translate 5 along new ŷ_b:
```
T_sb' = T_sb1 · | 1  0  0  0 |   |-1   0  0   4 || 1  0  0  0 |
                 | 0  1  0  5 | = | 0  -1  0   0 || 0  1  0  5 |
                 | 0  0  1  0 |   | 0   0  1   2 || 0  0  1  0 |
                 | 0  0  0  1 |   | 0   0  0   1 || 0  0  0  1 |

       = |-1   0  0   4 |
         | 0  -1  0  -5 |
         | 0   0  1   2 |
         | 0   0  0   1 |
```

The end-effector is now at position [4, −5, 2]^T with the x and y axes reversed.

---

## 7. Twists (Spatial Velocities)

### Concept

A **twist** V ∈ ℝ⁶ packages angular and linear velocity:

**Body twist**: V_b = [ω_b; v_b], where
- ω_b = angular velocity in {b}
- v_b = linear velocity of a point at {b} origin, expressed in {b}

**Spatial twist**: V_s = [ω_s; v_s], where
- ω_s = angular velocity in {s}
- v_s = linear velocity of a point at {s} origin (imagining body extends infinitely), expressed in {s}

**Matrix representations**:
```
[V_b] = T^{-1} Ṫ = | [ω_b]  v_b |  ∈ se(3)
                     |   0     0  |

[V_s] = Ṫ T^{-1} = | [ω_s]  v_s |  ∈ se(3)
                     |   0     0  |
```

**Important**: v_s ≠ ṗ in general! v_s = ṗ − ω_s × p = ṗ + ω_s × (−p)

### Practice Problems

**Easy**: A body rotates about ẑ at 2 rad/s. The body frame origin is at p = [3, 0, 0]^T in {s}, with R = I. Find V_b and V_s.

**Solution**:

ω_b = ω_s = [0, 0, 2]^T (since R = I)

v_b: velocity of {b} origin in {b} coordinates. Since the body rotates about ẑ_s passing through the origin of {s}, the {b} origin moves. But ω goes through {s} origin, not {b} origin:
v_b = ω_b × (−p expressed in {b}) ... 

Actually, let's use the definitions directly.

ṗ = ω_s × p = [0,0,2] × [3,0,0] = [0, 6, 0]

v_s = ṗ − ω_s × p = ṗ − ṗ ... that's wrong.

Let me re-derive. v_s = ṗ + ω_s × (−p):
v_s = [0, 6, 0] + [0, 0, 2] × [−3, 0, 0] = [0, 6, 0] + [0, −6, 0] ... hmm.

More carefully: v_s is the velocity of a point on the body currently at {s} origin.
The body rotates about ẑ_s through {s} origin, so that point has zero velocity.
v_s = [0, 0, 0]^T.

For V_s: ω_s = [0,0,2], v_s = ω_s × (−p_sb) + ṗ... 

Let me just use: v_s = ṗ − [ω_s]p = [0,6,0] − [0,0,2]×[3,0,0] = [0,6,0] − [0,6,0] = [0,0,0].

Wait, that gives v_s = 0, but that should be wrong for a rotation about the fixed-frame z-axis at the origin. Actually it's correct: the point on the body currently at {s} origin IS at the rotation axis, so its velocity IS zero.

V_s = [0, 0, 2, 0, 0, 0]^T

v_b = R^T ṗ = I · [0, 6, 0] = [0, 6, 0]

V_b = [0, 0, 2, 0, 6, 0]^T

**Verify via adjoint**: V_s = [Ad_{T_sb}] V_b:

```
[Ad_T] = | R      0   |   | I      0  |
         | [p]R   R   | = | [p]    I  |

[p] = | 0   0   0 |
      | 0   0  -3 |
      | 0   3   0 |

V_s = | I   0 || 0 |   | 0 |
      |[p]  I || 0 | = | 0 |
               | 2 |   | 2 |
               | 0 |   | 0+0 |
               | 6 |   | -6+6 |  = [0,0,2,0,0,0]^T  ✓
               | 0 |   | 0+0 |
```

---

**Medium**: Given the body twist V_c = [0, 0, 1, 0, 0, 0]^T in frame {c} at position q = [2, 1, 0]^T from the origin of {s}, the rotation axis passes through q. What is V_b = [ω, v]^T expressed in {b} where T_bc is known?

*[This type of problem is deferred to the Adjoint section below for proper treatment.]*

---

**Hard**: A rigid body has the configuration:

```
T_sb = | 1  0  0   2 |
       | 0  1  0   1 |
       | 0  0  1   0 |
       | 0  0  0   1 |
```

The angular velocity is ω = ẑ (screw axis through the {c}-frame origin at [2, 1, 0] in {s}).
Find V_b and V_s.

**Solution**:
R = I, p = [2, 1, 0]^T

ω_b = ω_s = [0, 0, 1]^T (since R = I)

The body rotates about ẑ through q = [2, 1, 0], so ṗ = ω × p = [0,0,1]×[2,1,0] = [−1, 2, 0].

v_s = ṗ + ω_s × (−p) = [−1, 2, 0] + [0,0,1] × [−2, −1, 0] = [−1, 2, 0] + [1, −2, 0] = [0, 0, 0]

Hmm, that gives v_s = 0 again because the rotation axis passes through the {s} origin... wait, the axis passes through [2,1,0], not through origin.

Let me redo: the screw axis is at q = [2,1,0] with ŝ = ẑ.

v_s = −ŝ × q = −[0,0,1] × [2,1,0] = −[−1, 2, 0] = [1, −2, 0]

Wait, for a screw with no pitch: v = −ω × q = ω × (−q):
v_s = [0,0,1] × [−2, −1, 0] = [−(−1)·0 − 0·0, 0·(−2) − 1·0, 1·(−1) − 0·(−2)]... 

Let me use the formula: for a zero-pitch screw at point q with axis ŝ:
S = [ŝ; −ŝ × q] = [ŝ; q × ŝ]... 

Using the formula v = −ω × q + hω (h=0): v = −ω × q

v_s = −[0,0,1] × [2,1,0] = −[0·0 − 1·1, 1·2 − 0·0, 0·1 − 0·2] = −[−1, 2, 0] = [1, −2, 0]

V_s = [0, 0, 1, 1, −2, 0]^T

v_b = R^T(ṗ) = ṗ = ω × p = [0,0,1] × [2,1,0] = [−1, 2, 0]

V_b = [0, 0, 1, −1, 2, 0]^T

Verify: V_s = [Ad_T] V_b:
```
[Ad_T] = | I   0 |   [p] = | 0   0  -1|
         |[p]  I |          | 0   0   2|... 
```

Actually [p] for p = [2,1,0]:
```
[p] = | 0   0   1 |
      | 0   0  -2 |
      |-1   2   0 |
```

Wait:
```
[p] = |  0   -p₃   p₂ |   |  0    0    1 |
      |  p₃   0   -p₁ | = |  0    0   -2 |
      | -p₂  p₁    0  |   | -1    2    0 |
```

V_s = | I   0 | |  0 |   |          0         |
      |[p]  I | |  0 | = |          0         |
                |  1 |   |          1         |
                | -1 |   | [p]ω_b + v_b       |
                |  2 |
                |  0 |

[p]ω_b = | 0  0  1 || 0 |   | 1 |
         | 0  0 -2 || 0 | = |-2 |
         |-1  2  0 || 1 |   | 0 |

v_s = [p]ω_b + v_b = [1,-2,0] + [-1,2,0] = [0, 0, 0]

That gives V_s = [0,0,1,0,0,0]... which seems wrong because the axis doesn't pass through origin.

I think I made an error above. Let me reconsider. The point on the body currently at the {s} origin: since R = I and p = [2,1,0], the body point at {s} origin corresponds to body coordinates [−2, −1, 0]. Its velocity = ω × q_axis_from_point... 

Actually, the issue is simpler. A rotation about ẑ through point q = [2,1,0]: the velocity of a point at the {s} origin (which is at distance √5 from the axis) should NOT be zero.

Let me reconsider. The formula v_s = ṗ + ω_s × (−p) from the textbook eq. 3.73:
ṗ is the velocity of the {b} origin.

If the body rotates about an axis through q = [2,1,0] in ẑ direction, the velocity of point p = [2,1,0] (which is on the axis) is zero. But the {b} origin IS at p = [2,1,0]. So ṗ = 0.

v_s = 0 + [0,0,1] × [−2,−1,0] = [−(−1), −(−2)·... ]

ω × (−p) = [0,0,1] × [−2,−1,0] = [(0)(0)−(1)(−1), (1)(−2)−(0)(0), (0)(−1)−(0)(−2)] = [1, −2, 0]

So V_s = [0, 0, 1, 1, −2, 0]^T.  And V_b = [0, 0, 1, 0, 0, 0]^T (since the rotation axis passes through {b} origin, v_b = 0).

Let me re-verify the adjoint:
[p]ω_b = [p][0,0,1]^T = [1, −2, 0]^T (from above)
v_s = [p]ω_b + v_b = [1,−2,0] + [0,0,0] = [1,−2,0]  ✓

So **V_s = [0, 0, 1, 1, −2, 0]^T** and **V_b = [0, 0, 1, 0, 0, 0]^T**.

---

## 8. Adjoint Representation

### Concept

The **adjoint map** [Ad_T] converts twists between frames:

```
V_s = [Ad_{T_sb}] V_b       V_b = [Ad_{T_bs}] V_s
```

For T = (R, p):

```
[Ad_T] = | R      0   |  ∈ ℝ^{6×6}
         | [p]R   R   |
```

**Key properties**:
- [Ad_{T₁}][Ad_{T₂}] = [Ad_{T₁T₂}]
- [Ad_T]^{-1} = [Ad_{T^{-1}}]

### Practice Problems

**Easy**: Compute [Ad_T] for T = (I, p) with p = [1, 0, 0]^T.

**Solution**:
```
[p] = | 0   0   0 |
      | 0   0  -1 |
      | 0   1   0 |

[Ad_T] = | I   0    |   | 1  0  0  0  0  0 |
         | [p]  I   | = | 0  1  0  0  0  0 |
                        | 0  0  1  0  0  0 |
                        | 0  0  0  1  0  0 |
                        | 0  0 -1  0  1  0 |
                        | 0  1  0  0  0  1 |
```

---

**Medium**: Given T_bc = (I, [2, 1, 0]^T), convert twist V_c = [0, 0, 1, 0, 0, 0]^T to V_b.

**Solution**:
V_b = [Ad_{T_bc}] V_c

```
[p] = | 0   0  -1 |        [p]R = [p]I = [p]
      | 0   0   2 |
      | 1  -1   0 |

Wait, p = [2, 1, 0]:
[p] = | 0    0    1 |
      | 0    0   -2 |
      |-1    2    0 |

Actually for p = [p₁, p₂, p₃] = [2, 1, 0]:
[p] = |  0   -0    1 |   | 0   0   1 |
      |  0    0   -2 | = | 0   0  -2 |
      | -1    2    0 |   |-1   2   0 |
```

```
[Ad_{T_bc}] = | I       0  |
              | [p]I    I  |

V_b = | I   0 || 0 |   |      0     |
      |[p]  I || 0 | = |      0     |
               | 1 |   |      1     |
               | 0 |   | [p]ω + v  |
               | 0 |
               | 0 |

[p]ω = | 0   0   1 || 0 |   | 1 |
       | 0   0  -2 || 0 | = |-2 |
       |-1   2   0 || 1 |   | 0 |

v_b = [1, -2, 0]^T + [0, 0, 0]^T = [1, -2, 0]^T
```

**V_b = [0, 0, 1, 1, −2, 0]^T**

Physical interpretation: The rotation axis (ẑ) passing through {c} origin appears, from {b}'s perspective, to also create a linear velocity component because the axis doesn't pass through {b}'s origin.

---

**Hard**: Given T_sb and T_sc, find [Ad_{T_bc}] and use it to convert V_c to V_b.

```
T_sb = | 0  -1  0   0 |      T_sc = | 1  0  0   2 |
       | 1   0  0   2 |             | 0  1  0   1 |
       | 0   0  1   0 |             | 0  0  1   0 |
       | 0   0  0   1 |             | 0  0  0   1 |
```

V_c = [0, 0, 1, 3, 0, 0]^T

**Solution**:

Step 1: Find T_bc = T_sb^{-1} · T_sc

```
T_sb^{-1}: R^T = | 0  1  0 |, -R^T p = -| 0  1  0 || 0 | = -| 2 | = |-2 |
                  |-1  0  0 |            |-1  0  0 || 2 |   | 0 |   | 0 |
                  | 0  0  1 |            | 0  0  1 || 0 |   | 0 |   | 0 |

T_sb^{-1} = | 0   1  0  -2 |
            |-1   0  0   0 |
            | 0   0  1   0 |
            | 0   0  0   1 |
```

```
T_bc = T_sb^{-1} · T_sc = | 0   1  0  -2 || 1  0  0  2 |
                           |-1   0  0   0 || 0  1  0  1 |
                           | 0   0  1   0 || 0  0  1  0 |
                           | 0   0  0   1 || 0  0  0  1 |

     = | 0   1  0  -2+1 |   | 0   1  0  -1 |
       |-1   0  0   0-2 | = |-1   0  0  -2 |
       | 0   0  1   0   |   | 0   0  1   0 |
       | 0   0  0   1   |   | 0   0  0   1 |
```

Step 2: Compute [Ad_{T_bc}]:
R_bc = upper-left 3×3, p_bc = [-1, -2, 0]^T

```
[p_bc] = |  0    0   -2 |
         |  0    0    1 |
         |  2   -1    0 |

[p_bc]R_bc = |  0    0   -2 || 0   1  0 |   | 0   0  -2 |
             |  0    0    1 ||-1   0  0 | = | 0   0   1 |
             |  2   -1    0 || 0   0  1 |   | 1  2   0 |
```

Wait, let me redo this carefully:
```
[p_bc] for p = [-1, -2, 0]:
[p] = |  0    -0   -2 |   | 0    0   -2 |
      |  0     0    1 | = | 0    0    1 |
      |  2    -1    0 |   | 2   -1    0 |

Hmm, p = [p₁, p₂, p₃] = [-1, -2, 0]:
[p] = |  0   -p₃   p₂  |   | 0    0   -2 |
      |  p₃   0   -p₁  | = | 0    0    1 |
      | -p₂   p₁   0   |   | 2   -1    0 |
```

```
[p]R = | 0    0   -2 || 0   1  0 |   | 0    0   -2 |
       | 0    0    1 ||-1   0  0 | = | 0    0    1 |
       | 2   -1    0 || 0   0  1 |   | 1    2    0 |
```

```
[Ad_{T_bc}] = | R_bc      0     |
              | [p]R_bc   R_bc  |

= |  0   1  0   0   0   0 |
  | -1   0  0   0   0   0 |
  |  0   0  1   0   0   0 |
  |  0   0 -2   0   1   0 |
  |  0   0  1  -1   0   0 |
  |  1   2  0   0   0   1 |
```

Step 3: V_b = [Ad_{T_bc}] V_c:

```
V_b = |  0   1  0   0   0   0 || 0 |   | 0  |
      | -1   0  0   0   0   0 || 0 |   | 0  |
      |  0   0  1   0   0   0 || 1 | = | 1  |
      |  0   0 -2   0   1   0 || 3 |   |-2  |
      |  0   0  1  -1   0   0 || 0 |   | 1-3|
      |  1   2  0   0   0   1 || 0 |   | 0  |

Wait let me compute each row:
Row 1: 0·0 + 1·0 + 0·1 + 0·3 + 0·0 + 0·0 = 0
Row 2: -1·0 + 0·0 + 0·1 + 0·3 + 0·0 + 0·0 = 0
Row 3: 0·0 + 0·0 + 1·1 + 0·3 + 0·0 + 0·0 = 1
Row 4: 0·0 + 0·0 + (-2)·1 + 0·3 + 1·0 + 0·0 = -2
Row 5: 0·0 + 0·0 + 1·1 + (-1)·3 + 0·0 + 0·0 = 1-3 = -2
Row 6: 1·0 + 2·0 + 0·1 + 0·3 + 0·0 + 1·0 = 0
```

**V_b = [0, 0, 1, −2, −2, 0]^T**

---

## 9. Screw Axes

### Concept

A **screw axis** S = [ω; v] ∈ ℝ⁶ is a normalized twist, where either:
- ‖ω‖ = 1 (finite pitch): v = −ω × q + hω, where q is a point on the axis and h is the pitch
- ω = 0, ‖v‖ = 1 (infinite pitch): pure translation

A general twist V = Sθ̇, where θ̇ is the rate of rotation (or translation for infinite pitch).

**For a revolute joint** (h = 0): S = [ω; −ω × q] = [ω; q × ω]
**For a prismatic joint** (h = ∞): S = [0; v̂] where v̂ is the unit translation direction

### Practice Problems

**Easy**: A revolute joint rotates about ẑ through the origin. Find S.

**Solution**:
ω = [0, 0, 1]^T, q = [0, 0, 0]^T, h = 0.

v = −ω × q = −[0,0,1] × [0,0,0] = [0, 0, 0]

**S = [0, 0, 1, 0, 0, 0]^T**

---

**Medium**: A revolute joint rotates about ẑ, and its axis passes through the point q = [3, 0, 0]^T. Find the space-frame screw axis S.

**Solution**:
ω = [0, 0, 1]^T, q = [3, 0, 0]^T, h = 0.

v = −ω × q = −[0, 0, 1] × [3, 0, 0] = −[0·0 − 1·0, 1·3 − 0·0, 0·0 − 0·3] = −[0, 3, 0] = [0, −3, 0]

**S = [0, 0, 1, 0, −3, 0]^T**

---

**Hard**: A robot has screw axes S₁, S₂, S₃ and home configuration M. Given:

```
S₁ = [0, 0, 1, 0, 0, 0]^T (revolute at origin about ẑ)
S₂ = [0, 0, 0, 0, -1, 0]^T (prismatic along -ŷ)
S₃ = [0, 0, 1, -L, -L, 0]^T (revolute about ẑ through [L, -L, 0])

M = | 1  0  0   2L |
    | 0  1  0   -L |
    | 0  0  1    0 |
    | 0  0  0    1 |
```

Find T_sb for θ = [0, L, −π/2].

**Solution**:
T_sb = e^{[S₁]θ₁} · e^{[S₂]θ₂} · e^{[S₃]θ₃} · M

For θ₁ = 0: e^{[S₁]·0} = I₄

For θ₂ = L (prismatic): 
```
e^{[S₂]L} = | I   v₂L | = | 1  0  0   0 |
             | 0    1  |   | 0  1  0  -L |
                            | 0  0  1   0 |
                            | 0  0  0   1 |
```

For θ₃ = −π/2 (revolute with ω = [0,0,1]):
```
e^{[ω]θ} = I + sin(−π/2)[ω̂] + (1−cos(−π/2))[ω̂]²

[ω̂] = | 0  -1  0 |     [ω̂]² = |-1  0  0 |
       | 1   0  0 |             | 0 -1  0 |
       | 0   0  0 |             | 0  0  0 |

e^{[ω](-π/2)} = I + (-1)[ω̂] + (1)[ω̂]² = | 0   1  0 |
                                              |-1   0  0 |
                                              | 0   0  1 |
```

For the translation part, G(θ)v with v = [-L, -L, 0]^T, θ = -π/2:
```
G(θ) = Iθ + (1−cosθ)[ω] + (θ−sinθ)[ω]²

= I(-π/2) + (1−0)[ω] + (−π/2−(−1))[ω]²

= (-π/2)I + [ω] + (1−π/2)[ω]²

Hmm, let me be more careful:
θ = -π/2, sinθ = -1, cosθ = 0

G(θ) = (-π/2)I + (1-0)| 0 -1 0| + (-π/2-(-1))|-1 0 0|
                       | 1  0 0|               | 0 -1 0|
                       | 0  0 0|               | 0  0 0|

= (-π/2)I + | 0 -1 0| + (1-π/2)|-1  0 0|
             | 1  0 0|           | 0 -1 0|
             | 0  0 0|           | 0  0 0|

= |π/2-1+π/2   -1       0  |... 
```

This is getting complex. Let me use the full matrix exponential formula directly.

For S₃ = [0, 0, 1, −L, −L, 0]^T:

```
[S₃] = | [ω]  v |   | 0  -1  0  -L |
       |  0   0 | = | 1   0  0  -L |
                    | 0   0  0   0 |
                    | 0   0  0   0 |
```

e^{[S₃]θ₃} with θ₃ = −π/2:

The rotation part is e^{[ω](-π/2)} = | 0  1  0 |
                                       |-1  0  0 |
                                       | 0  0  1 |

The translation part G(θ)v:
```
G(θ) = Iθ + (1−cosθ)[ω] + (θ − sinθ)[ω]²

θ = -π/2, sin(-π/2) = -1, cos(-π/2) = 0

G = (-π/2)I + (1)| 0 -1 0| + (-π/2+1)|-1  0  0|
                  | 1  0 0|            | 0 -1  0|
                  | 0  0 0|            | 0  0  0|

= |-π/2+π/2-1    -1         0  |   |-1   -1    0 |
  | 1          -π/2+π/2-1   0  | = | 1   -1    0 |
  | 0             0       -π/2 |   | 0    0  -π/2|
```

G(θ)v = |-1  -1    0 ||-L|   | L+L  |   | 2L  |
        | 1  -1    0 ||-L| = |-L+L  | = |  0  |
        | 0   0  -π/2|| 0|   |  0   |   |  0  |

So:
```
e^{[S₃](-π/2)} = | 0   1  0  2L |
                  |-1   0  0   0 |
                  | 0   0  1   0 |
                  | 0   0  0   1 |
```

Now multiply: T_sb = I · e^{[S₂]L} · e^{[S₃](-π/2)} · M

```
Step 1: e^{[S₂]L} · e^{[S₃](-π/2)}

= | 1  0  0   0 || 0   1  0  2L |   | 0   1  0   2L |
  | 0  1  0  -L ||-1   0  0   0 | = |-1   0  0   -L |
  | 0  0  1   0 || 0   0  1   0 |   | 0   0  1    0 |
  | 0  0  0   1 || 0   0  0   1 |   | 0   0  0    1 |
```

```
Step 2: × M

= | 0   1  0   2L || 1  0  0  2L |   | 0  1  0  2L-L |   | 0  1  0   L  |
  |-1   0  0   -L || 0  1  0  -L | = |-1  0  0 -L-2L| = |-1  0  0  -3L |
  | 0   0  1    0 || 0  0  1   0 |   | 0  0  1   0   |   | 0  0  1    0 |
  | 0   0  0    1 || 0  0  0   1 |   | 0  0  0   1   |   | 0  0  0    1 |
```

**Final answer**:
```
T_sb = | 0   1  0    L |
       |-1   0  0  -3L |
       | 0   0  1    0 |
       | 0   0  0    1 |
```

Position of end-effector: [L, −3L, 0]^T
Orientation: x̂_b = [0,−1,0], ŷ_b = [1,0,0], ẑ_b = [0,0,1] (90° rotation about ẑ, CCW from body perspective → CW from space perspective).

---

## 10. Exponential Coordinates for Rigid-Body Motions

### Concept

The six-dimensional exponential coordinates Sθ ∈ ℝ⁶ define a screw motion. The matrix exponential maps se(3) → SE(3):

**If ‖ω‖ = 1** (finite pitch):
```
e^{[S]θ} = | e^{[ω]θ}    G(θ)v |
           |    0           1   |

G(θ) = Iθ + (1 − cosθ)[ω] + (θ − sinθ)[ω]²
```

**If ω = 0, ‖v‖ = 1** (pure translation):
```
e^{[S]θ} = | I    vθ |
           | 0     1 |
```

### Practice Problems

**Easy**: Compute e^{[S]θ} for S = [0, 0, 0, 1, 0, 0]^T and θ = 5.

**Solution**:
ω = [0,0,0], v = [1,0,0], ‖v‖ = 1. Pure translation case:

```
e^{[S]·5} = | 1  0  0  5 |
            | 0  1  0  0 |
            | 0  0  1  0 |
            | 0  0  0  1 |
```

This is a translation of 5 units along the x-axis.

---

**Medium**: Compute e^{[S]θ} for S = [0, 0, 1, 0, 0, 0]^T, θ = π/2.

**Solution**:
ω = [0,0,1], v = [0,0,0]. Rotation about ẑ through origin.

```
e^{[ω]π/2} = Rot(ẑ, π/2) = | 0  -1  0 |
                              | 1   0  0 |
                              | 0   0  1 |

G(π/2) = (π/2)I + (1−cos(π/2))[ω] + (π/2−sin(π/2))[ω]²

= (π/2)I + (1)| 0 -1 0| + (π/2−1)|-1  0  0|
               | 1  0 0|           | 0 -1  0|
               | 0  0 0|           | 0  0  0|

G(π/2)v = G(π/2)[0,0,0]^T = [0,0,0]^T
```

```
e^{[S]π/2} = | 0  -1  0  0 |
             | 1   0  0  0 |
             | 0   0  1  0 |
             | 0   0  0  1 |
```

Pure rotation about ẑ by 90°, no translation.

---

**Hard**: A link rotates about the z-axis through {s} origin. Frame {b} is initially at position [1, 0, 0] with frames aligned. Find T_sb after rotating by θ = π/2.

S = [0, 0, 1, 0, 0, 0]^T (screw axis), M = T_sb(0):

```
M = | 1  0  0  1 |
    | 0  1  0  0 |
    | 0  0  1  0 |
    | 0  0  0  1 |
```

**Solution**:

T_sb = e^{[S]π/2} · M

From the medium problem above:
```
e^{[S]π/2} = | 0  -1  0  0 |
             | 1   0  0  0 |
             | 0   0  1  0 |
             | 0   0  0  1 |

T_sb = | 0  -1  0  0 || 1  0  0  1 |   | 0  -1  0  0 |
       | 1   0  0  0 || 0  1  0  0 | = | 1   0  0  1 |
       | 0   0  1  0 || 0  0  1  0 |   | 0   0  1  0 |
       | 0   0  0  1 || 0  0  0  1 |   | 0   0  0  1 |
```

The {b} origin moved from [1,0,0] to [0,1,0] (quarter circle), and the frame rotated 90° about ẑ.

---

## 11. Matrix Logarithm of Rigid-Body Motions

### Concept

Given T = (R, p) ∈ SE(3), find S and θ such that e^{[S]θ} = T.

**Algorithm**:

**(a)** If R = I: ω = 0, v = p/‖p‖, θ = ‖p‖.

**(b)** Otherwise: use the SO(3) matrix logarithm to find ω̂ and θ from R, then:
```
v = G^{-1}(θ) p

G^{-1}(θ) = (1/θ)I − (1/2)[ω] + (1/θ − (1/2)cot(θ/2))[ω]²
```

### Practice Problems

**Easy**: Find the matrix logarithm of:

```
T = | 1  0  0  3 |
    | 0  1  0  4 |
    | 0  0  1  0 |
    | 0  0  0  1 |
```

**Solution**:
R = I, so case (a) applies.

p = [3, 4, 0]^T,  ‖p‖ = 5

v = p/‖p‖ = [3/5, 4/5, 0]^T,  θ = 5

**S = [0, 0, 0, 3/5, 4/5, 0]^T, θ = 5**

---

**Medium**: Find the exponential coordinates for:

```
T = | 0  -1  0  3 |
    | 1   0  0  0 |
    | 0   0  1  0 |
    | 0   0  0  1 |
```

**Solution**:

R ≠ I. First, find ω̂, θ from R:
```
R = | 0  -1  0 |
    | 1   0  0 |
    | 0   0  1 |
```

tr(R) = 0 + 0 + 1 = 1. θ = cos⁻¹((1−1)/2) = cos⁻¹(0) = π/2.

```
[ω̂] = (1/(2sin(π/2)))(R − R^T) = (1/2)| 0  -2   0 |   | 0  -1  0 |
                                         | 2   0   0 | = | 1   0  0 |
                                         | 0   0   0 |   | 0   0  0 |
```

So ω̂ = [0, 0, 1]^T, θ = π/2. (Rotation about ẑ by 90°.)

Now find v = G⁻¹(θ) p:

```
G⁻¹(π/2) = (2/π)I − (1/2)| 0 -1 0| + (2/π − 1/2·cot(π/4))| -1  0  0|
                           | 1  0 0|                          |  0 -1  0|
                           | 0  0 0|                          |  0  0  0|

cot(π/4) = 1

= (2/π)I − (1/2)[ω] + (2/π − 1/2)[-1  0  0; 0 -1 0; 0 0 0]

= | 2/π + (−2/π+1/2)   1/2              0   |
  |-1/2                 2/π+(−2/π+1/2)   0   |
  | 0                   0                2/π |

= | 1/2    1/2     0  |
  |-1/2    1/2     0  |
  | 0       0     2/π |

v = G⁻¹ p = | 1/2    1/2     0 || 3 |   | 3/2 |
             |-1/2    1/2     0 || 0 | = |-3/2 |
             | 0       0    2/π || 0 |   |  0  |
```

**Sθ = [0, 0, π/2, 3/2, −3/2, 0]^T**

Or: S = [0, 0, 1, 3/π, −3/π, 0]^T, θ = π/2.

---

**Hard**: This is equivalent to finding the screw axis of the displacement in the medium problem. The screw axis passes through some point q with ŝ = [0,0,1] and pitch h = 0 (since v·ω = 0). Find q.

**Solution**:
From S = [ω; v] = [0, 0, 1, 3/π, −3/π, 0]^T:

For zero pitch: v = −ω × q, or equivalently v = q × ω.

```
[0, 0, 1] × [qx, qy, qz] = [-qy, qx, 0]
```

So: −(−qy) = 3/π → qy = 3/π
    −qx = −3/π → qx = 3/π

**q = [3/π, 3/π, 0]^T ≈ [0.955, 0.955, 0]^T**

This makes geometric sense: the rotation is 90° about ẑ through a point approximately at (0.955, 0.955), which maps the origin of {b} at [0,0,0] in {s} after rotation to approximately [3,0,0] in {s}.

---

## 12. Wrenches

### Concept

A **wrench** F = [m; f] ∈ ℝ⁶ packages moment m and force f:

```
F_a = [m_a; f_a]
```

**Change of frame** (uses transpose of adjoint):
```
F_b = [Ad_{T_ab}]^T F_a
F_a = [Ad_{T_ba}]^T F_b
```

**Power conservation**: V_b^T F_b = V_a^T F_a (power is frame-independent).

### Practice Problems

**Easy**: A force f = [0, 0, −10]^T N acts at the origin of frame {b} (gravity). Frame {b} is at {s} origin with R = I. What is the wrench F_s?

**Solution**:
Since R = I and p = 0, the wrench in {s} is:
F_s = [m_s; f_s] = [0; f] = [0, 0, 0, 0, 0, −10]^T

(No moment because the force acts at the {s} origin.)

---

**Medium**: A 1 kg mass hangs at the end-effector. The end-effector frame {b} has its ẑ_b pointing down (aligned with −ẑ_s). Frame {b} is at p = [2, 0, 0]^T from {s}.

```
R_sb = | 1   0   0 |
       | 0  -1   0 |
       | 0   0  -1 |
```

The gravitational force in {s} is f_s = [0, 0, −10]^T N. Find F_s and F_b.

**Solution**:

F_s (spatial wrench): The moment at {s} origin due to force at p:
m_s = p × f_s = [2,0,0] × [0,0,−10] = [0·(−10)−0·0, 0·0−2·(−10), 2·0−0·0] = [0, 20, 0]

F_s = [0, 20, 0, 0, 0, −10]^T

F_b (body wrench): The force in {b} coordinates:
f_b = R^T f_s = [0, 0, 10]^T (since ẑ_b = −ẑ_s, force is +10 in ẑ_b)

The moment at {b} origin is zero (force acts at {b} origin):
F_b = [0, 0, 0, 0, 0, 10]^T

---

**Hard**: Verify the wrench transformation F_b = [Ad_{T_sb}]^T F_s for the medium problem above using the adjoint.

**Solution**:

```
T_sb = | 1   0   0  2 |
       | 0  -1   0  0 |
       | 0   0  -1  0 |
       | 0   0   0  1 |

[p] = | 0   0   0 |
      | 0   0  -2 |
      | 0   2   0 |

[Ad_{T_sb}] = | R       0   |   
              | [p]R    R   |

R = | 1   0   0 |
    | 0  -1   0 |
    | 0   0  -1 |

[p]R = | 0   0   0 || 1   0   0 |   | 0   0   0 |
       | 0   0  -2 || 0  -1   0 | = | 0   0   2 |
       | 0   2   0 || 0   0  -1 |   | 0  -2   0 |
```

```
[Ad_{T_sb}]^T = | R^T     ([p]R)^T |
                |  0        R^T    |

= | 1   0   0   0   0   0 |
  | 0  -1   0   0   0  -2 |
  | 0   0  -1   0   2   0 |
  | 0   0   0   1   0   0 |
  | 0   0   0   0  -1   0 |
  | 0   0   0   0   0  -1 |
```

Wait, let me be more careful with the transpose:

```
[Ad_T] = | R      0  |  (6×6 matrix, R is 3×3 blocks)
         | [p]R   R  |

[Ad_T]^T = | R^T      ([p]R)^T |
           |  0          R^T   |
```

F_b = [Ad_{T_sb}]^T F_s:

```
F_s = [0, 20, 0, 0, 0, -10]^T

Top 3: R^T m_s + ([p]R)^T f_s
Bottom 3: R^T f_s

R^T m_s = | 1   0   0 || 0 |   |  0 |
          | 0  -1   0 || 20| = |-20 |
          | 0   0  -1 || 0 |   |  0 |

([p]R)^T f_s = | 0   0   0 |^T | 0 |   | 0   0   0 || 0  |   |  0 |
               | 0   0   2 |   | 0 | = | 0   0  -2 || 0  | = | 20 |
               | 0  -2   0 |   |-10|   | 0   2   0 ||-10 |   |  0 |

Top 3: [0, -20, 0] + [0, 20, 0] = [0, 0, 0]  ✓

R^T f_s = | 1   0   0 || 0  |   |  0 |
          | 0  -1   0 || 0  | = |  0 |
          | 0   0  -1 ||-10 |   | 10 |

Bottom 3: [0, 0, 10]  ✓
```

**F_b = [0, 0, 0, 0, 0, 10]^T** — matches our earlier result!  ✓

---

## Key Formulas Quick Reference

| Concept | Formula |
|---------|---------|
| Rotation matrix inverse | R⁻¹ = R^T |
| Rodrigues' formula | R = I + sinθ[ω̂] + (1−cosθ)[ω̂]² |
| Angle from R | θ = cos⁻¹((tr(R)−1)/2) |
| Axis from R | [ω̂] = (R−R^T)/(2sinθ) |
| Transformation inverse | T⁻¹ = [R^T, −R^T p; 0, 1] |
| Space twist | [V_s] = ṪT⁻¹ |
| Body twist | [V_b] = T⁻¹Ṫ |
| Adjoint | [Ad_T] = [R, 0; [p]R, R] |
| Twist conversion | V_s = [Ad_{T_sb}]V_b |
| Wrench conversion | F_b = [Ad_{T_ab}]^T F_a |
| Matrix exp (SE(3)) | e^{[S]θ} = [e^{[ω]θ}, G(θ)v; 0, 1] |
| G(θ) | Iθ + (1−cosθ)[ω] + (θ−sinθ)[ω]² |
| G⁻¹(θ) | (1/θ)I − ½[ω] + (1/θ − ½cot(θ/2))[ω]² |
| Pre-multiply R | Rotation about **fixed** frame axis |
| Post-multiply R | Rotation about **body** frame axis |
| Revolute screw (h=0) | S = [ω̂; −ω̂ × q] |
| Prismatic screw | S = [0; v̂] |
| Skew-symmetric identity | [ω̂]³ = −[ω̂] |
| Conjugation identity | R[ω]R^T = [Rω] |

---

*End of Chapter 3 Exam Preparation Notes*