# 7-DoF KUKA LWR — Redundancy-Aware Kinematic Control in Julia

> **Academic project** · Polytech Montpellier · Robotics S7 · Jan. 2026  
> Full report: [`Rapport_KUKA_LWR.pdf`](./Rapport_KUKA_LWR.pdf)

---

## Overview

This project implements and progressively extends operational-space kinematic control for the **KUKA LWR 7-DoF redundant manipulator**, simulated in **CoppeliaSim**. Everything is written in **Julia** and communicates with the simulator through the legacy Remote API.

The extra degree of freedom relative to a minimal 6-DoF arm opens up a null-space that can be exploited for secondary objectives. This project demonstrates that clearly through three incremental steps:

1. **Position-only kinematic control** — move the end-effector to a Cartesian target while only minimising position error.
2. **Null-space joint-limit avoidance** — add a projection term in the Jacobian null-space that pushes all joints away from their mechanical stops, without disturbing the primary Cartesian task.
3. **Full pose control (position + orientation)** — extend the Jacobian to 6 rows and regulate orientation error with a damped pseudo-inverse to handle singularities, while keeping null-space joint-limit avoidance active.

---

## Repository Structure

```
.
├── main.jl              # All control logic: MGD wrapper, two control loops, null-space term
├── lib-robotique.jl     # Robot kinematics library: DH model, MGD, Jacobian, RTL, CreateRobotKukaLwr
├── lib-CSim.jl          # CoppeliaSim Remote API Julia bindings
├── KUKA-MEA4.ttt        # CoppeliaSim scene
├── remoteApi.so         # CoppeliaSim Remote API shared library (Linux)
└── Rapport_KUKA_LWR.pdf # Full project report
```

---

## Getting Started

### Prerequisites

- **Julia ≥ 1.9**
- **CoppeliaSim 4.4** (free EDU version works)
- Required Julia packages:

```julia
using Pkg
Pkg.add(["LinearAlgebra", "Plots"])
```

### CoppeliaSim setup

1. Open `KUKA-MEA4.ttt` in CoppeliaSim.
2. Make sure the simulator is **not** running yet — `main.jl` starts it.
3. The legacy Remote API server must be enabled on port `5555` (pre-configured in the scene).

### Running

```bash
julia main.jl
```

The active scenario is selected by commenting/uncommenting two lines near the bottom of `main.jl`:

```julia
# Position-only control (no orientation):
# q_collect = inverseKinematicsPos(θinit, Pb, rob, dP, dt, -2)

# Full pose control (position + orientation + null-space avoidance):
q_collect, epsilon0_collect = inverseKinematics(θinit, Pb, Ab, rob, dP, dω, dt, -2)
```

Plot output is selected the same way — uncomment the relevant block at the end of the file to display joint trajectories, end-effector path, or orientation error convergence.

---

## Implementation Details

### Robot model — `lib-robotique.jl`

`CreateRobotKukaLwr()` constructs the DH parameter struct for the KUKA LWR. The `MGD` function returns the full chain of homogeneous transforms and the end-effector pose `Te`. The geometric Jacobian `Jacobian(q, rob, Pe)` is built column by column using the standard cross-product formulation.

KUKA LWR joint limits used:

| Joint | θ_min (°) | θ_max (°) | ω_max (°/s) |
|-------|-----------|-----------|-------------|
| 1     | −170      | 170       | 112.5       |
| 2     | −120      | 120       | 112.5       |
| 3     | −170      | 170       | 112.5       |
| 4     | −120      | 120       | 112.5       |
| 5     | −170      | 170       | 180         |
| 6     | −120      | 120       | 112.5       |
| 7     | −170      | 170       | 112.5       |

### Position-only control — `inverseKinematicsPos`

Resolved-rate control law restricted to the 3-DoF position task:

```
dq = J₃⁺ · (Kp·εp + dPd) + (I − J₃⁺J₃) · α·∇φ
```

where `J₃` is the top 3 rows of the geometric Jacobian.

### Full pose control — `inverseKinematics`

Extends the Jacobian to 6 rows (linear + angular velocity). Orientation error is computed from the rotation matrix cross-product formulation:

```
ε₀ = ½ (sₑ×sₐ + nₑ×nₐ + aₑ×aₐ)
```

The interaction matrix `L` maps orientation error to angular velocity. Because `L` becomes ill-conditioned as the actual orientation `Aₑ` approaches the desired `Aₐ`, a **damped pseudo-inverse** is used:

```
L⁺_damped = Lᵀ · (L·Lᵀ + λ²·I)⁻¹
```

This keeps the orientation command bounded and stable near convergence.

### Null-space joint-limit avoidance — `eloignement_butees_articulaires`

The redundancy is exploited through a gradient projection onto the Jacobian null-space:

```
θ̇ = J⁺Ẋ + (I − J⁺J) · α · ∇φ
```

The potential function and its gradient per joint:

```
φᵢ(θ) = (θᵢ − θᵢ,moy)² / θ²ᵢ,range

∇φᵢ = 2(θᵢ − θᵢ,moy) / θ²ᵢ,range

where  θᵢ,moy = (θᵢ,max + θᵢ,min) / 2
       θᵢ,range = θᵢ,max − θᵢ,min
```

With `α = −2`, the gradient descent term drives every joint toward its midrange, while the projector `(I − J⁺J)` guarantees that this motion lies entirely in the null-space and does not affect the end-effector pose.

---

## Results

| Scenario | Observation |
|---|---|
| Position-only | End-effector tracks the z-axis path accurately; only 3 of 7 joints move (minimum-norm solution) |
| + Null-space avoidance | End-effector path unchanged; previously idle joints now drift toward their midrange |
| + Orientation control | All three components of ε₀ converge to zero; full pose achieved while maintaining null-space behaviour |

---

## Project Report

Full derivations, block diagrams, trajectory plots and simulation screenshots are in [`Rapport_KUKA_LWR.pdf`](./Rapport_KUKA_LWR.pdf).

---

## Author

**Lucas Musumeci** — Polytech Montpellier, MEA department
