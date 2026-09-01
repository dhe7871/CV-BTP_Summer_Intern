# Singularity-Free Vector-Field-Based Path Following

> A MATLAB-based study and implementation of higher-dimensional guiding vector fields for global, singularity-free path following, with extensions toward wind-aware navigation, curvature-constrained motion, path-parameter initialization, and obstacle-avoidance field modulation.

## Overview

This repository contains the implementation and experimentation developed during the project **CV-BTP / Summer Internship** on **singularity-free vector-field (VF) based path following**.

The central problem is a fundamental limitation of conventional vector-field-guided path-following (VF-PF) methods: when a desired path is **simple closed** (for example, a circle) or **self-intersected** (for example, a figure-eight), a time-invariant vector field of the same dimension as the physical workspace cannot, in general, provide globally convergent path-following behavior without encountering singularities.

The project follows the higher-dimensional construction proposed in the reference work:

> **W. Yao, H. Garcia de Marina, B. Lin, and M. Cao, “Singularity-free Guiding Vector Field for Robot Navigation.”**

The attached reference paper establishes the topological obstruction and introduces the key idea of augmenting the physical path with an additional virtual coordinate. This converts a problematic bounded/self-intersected physical path into an **unbounded, non-self-intersected curve in a higher-dimensional space**, where a singularity-free guiding vector field can be constructed. The project then turns this theoretical idea into a practical MATLAB simulation framework and investigates additional engineering considerations. The reference paper is included in the repository's `Reading Material` directory. fileciteturn0file0

The repository is therefore not simply a reproduction of the paper. It includes a practical simulation architecture around the higher-dimensional VF formulation, including multiple path families, a Chebyshev-based numerical root solver, initialization of the virtual path parameter, wind-aware ground-velocity computation, two navigation-controller formulations, curvature/course-rate constraints, extensive visualization tooling, and an exploratory obstacle-avoidance modulation field.

---

## Core Motivation

### The conventional VF-PF problem

For a desired path represented implicitly by constraints

$$
P = \{\xi \in \mathbb{R}^n : \phi_i(\xi)=0,\; i=1,\dots,n-1\},
$$

a conventional guiding vector field can be written conceptually as

$$
\chi(\xi)
=
\underbrace{\nabla\!\times\!\phi}_{\text{propagation / tangential term}}
-
\underbrace{N(\xi)K e(\xi)}_{\text{convergence term}},
$$

where `e` contains the path errors and the generalized cross product generates a direction tangent to all defining hypersurfaces.

This formulation is attractive because it simultaneously:

1. pushes the robot toward the desired path;
2. gives motion along the path once near it;
3. does not require a unique closest point on the path;
4. can be implemented using only local evaluations of the vector field.

However, the reference paper proves that a time-invariant vector field of the same dimension as the physical path cannot guarantee global convergence to every desired path. In particular:

- a **self-intersection becomes a singular point** because the path has no unique tangent direction at the crossing;
- a **simple closed path cannot be globally asymptotically attractive** under the considered autonomous formulation because of the topology of the ambient Euclidean space.

The paper formalizes this as its **Theorem 1 (Impossibility of global convergence)**. fileciteturn0file0

### The higher-dimensional solution

The key idea is to augment the path with an additional virtual coordinate `w`.

For a physical parameterized path

$$
x_i=f_i(w), \qquad w\in\mathbb{R},
$$

define the higher-dimensional constraints

$$
\phi_i(x_1,\dots,x_n,w)=x_i-f_i(w).
$$

The physical `n`-dimensional path becomes an `(n+1)`-dimensional curve

$$
P_{hgh}=\{(x_1,\dots,x_n,w):x_i=f_i(w)\}.
$$

The additional coordinate effectively **cuts and stretches** the physical curve. A closed or self-intersected physical path can therefore become an unbounded, non-self-intersected curve in the augmented space.

For the construction above, the propagation component has a non-zero constant component in the new dimension. Consequently, the higher-dimensional field is never zero, so its singular set is empty. The transformed trajectory can then be projected back onto the physical coordinates.

This is the conceptual foundation of the implementation in this repository. The reference paper provides the formal proof and derives the generalized `(n+1)`-dimensional field and global convergence result in its Theorem 2. fileciteturn0file0

---

## What This Repository Implements

The current implementation is organized around a 3-state augmented simulation for planar paths:

$$
\boxed{\xi=[x,y,w]^T}
$$

where:

- `x, y` are physical coordinates;
- `w` is the virtual/path-parameter coordinate.

The project supports several path classes directly in `main.m`, including:

- ellipse/circle;
- straight line;
- parabola;
- an 8-shaped self-intersecting curve.

The current active example is an **8-shaped curve**:

$$
 x(w)=a\sin(\beta w),
$$

$$
 y(w)=a\sin(\beta w)\cos(\beta w).
$$

The corresponding reference implementation is visible in `MATLAB/main.m`. fileciteturn4file0

The project goes beyond merely plotting the field. It simulates the motion of a vehicle under a wind-aware kinematic model and compares the desired vector-field direction with a realizable velocity vector while respecting a course-rate/curvature constraint.

---

## Mathematical Formulation Used in the Code

### 1. Parameterized physical path

The user specifies a physical path in parametric form:

$$
P_{phy}(w)=
\begin{bmatrix}
f_1(w)\\
f_2(w)
\end{bmatrix}.
$$

### 2. Higher-dimensional constraints

The implementation constructs

$$
\phi_1(x,y,w)=x-f_1(w),
$$

$$
\phi_2(x,y,w)=y-f_2(w).
$$

Thus, the higher-dimensional desired curve is the intersection

$$
\phi_1=0,\qquad \phi_2=0.
$$

This is exactly the 2D-to-3D construction described in the reference paper. fileciteturn0file0

### 3. Higher-dimensional vector field

The field combines a propagation component based on the derivatives `df_i/dw` with converging terms based on the constraint errors.

In the repository, `createVFAtPointMatFunc.m` evaluates a practical scaled/smoothed form of the 3D field. The implementation uses hyperbolic functions:

- `tanh(kappa_i * phi_i)` for bounded converging terms;
- `sech(kappa_1*phi_1) sech(kappa_2*phi_2)` as a smooth scaling of the propagation component.

This is an important implementation choice because it differs from simply coding the unmodified theoretical field. The function therefore acts as the numerical bridge between the mathematical construction and the simulation. fileciteturn5file0

### 4. Normalized desired direction

The field returned at the vehicle position is normalized before being used as the desired velocity direction. This separates the problem of **direction generation** from the problem of **realizing a physically feasible vehicle velocity**.

### 5. Wind-aware ground velocity

The simulator assumes a specified airspeed and non-zero wind. The ground-speed magnitude is calculated from the wind and course angle, allowing the vehicle trajectory to respond to environmental flow while the controller acts primarily on the course direction.

`main.m` currently demonstrates this with:

- airspeed `va = 15 m/s`;
- wind `[3, 4] m/s`;
- gravity `g = 9.81 m/s²`;
- maximum bank angle `phiMax = pi/4`.

These values are explicitly configured in the simulation script. fileciteturn4file0

---

## Vehicle / Navigation Model

The project explores two ways of turning the vector field into a feasible velocity command.

### A. Velocity-vector controller

`navVelocityController.m` aligns the current velocity direction with the desired vector direction using a cross-product based angular correction. It then limits the resulting planar curvature according to a specified maximum-curvature condition. fileciteturn17file0

Conceptually:

$$
\hat v \rightarrow \hat v_d
$$

while maintaining an allowable turning envelope.

The implementation computes an angular correction using

$$
\omega \propto \hat v\times \hat v_d
$$

and then scales the correction if the resulting curvature would exceed the prescribed bound. This controller is useful when the simulation is expressed in terms of velocity-vector dynamics rather than directly manipulating course angle.

### B. Course-distribution controller

The current default simulation selects the **`Dist`** controller. This controller converts the desired field direction into a desired course angle and applies a first-order course-rate correction:

$$
\dot\chi
=
 k_\chi\operatorname{wrapToPi}(\chi_d-\chi).
$$

The course rate is then explicitly saturated by

$$
|\dot\chi|\le \frac{g\tan\phi_{max}}{v_a}.
$$

After updating the course angle, the code recomputes the wind-consistent ground-speed magnitude and generates a new velocity vector. This makes the controller especially relevant for fixed-wing-style motion where heading/course dynamics and turning limitations cannot be ignored. fileciteturn7file0

---

## Why the Course-Rate Constraint Matters

For a fixed-wing platform, the mathematically ideal vector field may demand an arbitrarily sharp turn near a high-curvature portion of the path.

A real aircraft cannot execute arbitrary curvature. The project therefore tracks the realized planar curvature and compares it with an admissible bound.

In the default `Dist` architecture, curvature is derived from

$$
\kappa_{xy}=\frac{\dot\chi}{V_g},
$$

while the corresponding maximum course-rate is set from the bank-angle limit. `plotParams.m` explicitly visualizes both the measured curvature and the corresponding bounds. fileciteturn20file0

This engineering layer is essential because a field can be mathematically valid but physically unrealizable.

---

## Virtual Coordinate and the Path-Following Interpretation

The additional coordinate `w` is not just a mathematical trick. In the simulation it becomes a **dynamic internal state**.

The point

$$
P(w)=\big(f_1(w),f_2(w)\big)
$$

can be interpreted as a moving virtual target point on the desired curve.

Unlike a conventional trajectory-tracking reference, however, `w` is not simply a monotonic clock. It is coupled to the vehicle's current state through the vector-field dynamics. Thus the virtual point can move in response to where the vehicle currently is relative to the path.

This is consistent with the reference paper's interpretation of the higher-dimensional method as lying between conventional VF path following and trajectory tracking: `w` plays a time-like role, but its evolution is state-dependent rather than prescribed purely as a function of time. fileciteturn0file0

---

## Initialization of the Virtual Parameter

A non-trivial part of the implementation is determining a meaningful initial value `w0` from the vehicle's initial physical position.

Naively choosing `w0 = 0` can place the virtual target far from the actual vehicle even when the vehicle is already close to another point on the curve.

The function `initializeCurveParameter.m` therefore searches for candidate stationary points of the squared distance between the physical initial point and the parameterized curve. It:

1. constructs the first-order stationarity condition;
2. computes its roots over the selected parameter interval;
3. filters to local minima using a second-order test;
4. evaluates the actual squared distance at the candidate minima;
5. selects the closest candidate (with a deterministic tie-break rule).

The implementation delegates the nonlinear root-finding step to `chebyRoots.m`. fileciteturn12file0

This initialization makes the simulation substantially more robust because the augmented-state description depends on both physical position and virtual phase.

---

## Chebyshev-Based Root Solver

One of the more technically substantial utilities in the repository is `chebyRoots.m`.

Instead of relying exclusively on direct symbolic polynomial root extraction or generic nonlinear root search, the function supports a broader class of one-variable symbolic functions by:

1. detecting ordinary polynomials and solving them directly when possible;
2. mapping a finite interval to `[-1,1]`;
3. approximating non-polynomial functions using Chebyshev expansions;
4. increasing the polynomial degree until an approximation-error test is satisfied;
5. performing a midpoint consistency test to guard against false convergence / aliasing;
6. splitting the interval recursively when the approximation is inadequate;
7. converting the Chebyshev representation into a colleague matrix;
8. obtaining candidate roots from the eigenvalues of that matrix;
9. filtering approximately-real roots;
10. refining candidate roots with `fzero`.

The implementation also prunes numerical noise in the final Chebyshev coefficients before building the colleague matrix. fileciteturn10file0

This solver is primarily used by the curve-parameter initialization routine, but it is also a useful standalone numerical-analysis component of the project.

---

## Visualization Architecture

The repository contains a dedicated visualization layer rather than embedding all plotting directly in the simulation loop.

### `plotVF.m`

This utility visualizes both:

- the **physical 2D curve**;
- the **higher-dimensional 3D curve** `(x,y,w)`.

It can also evaluate and draw the vector field on selected planes through the augmented space. This makes it possible to inspect how the projected 2D field changes as the virtual coordinate varies. fileciteturn13file0

This visualization directly reflects an important conceptual property of the method: the system can be viewed as a family of projected vector fields whose geometry changes with `w`.

### `initializePlotTrajectory.m`

This creates synchronized 2D and 3D views showing:

- desired physical path;
- desired higher-dimensional curve;
- initial position;
- current vehicle position;
- current virtual target point;
- actual followed path.

The plot is updated during the simulation, so the evolution of both physical and augmented trajectories can be inspected in real time. fileciteturn19file0

### `plotParams.m`

After the simulation, this function plots dynamic quantities such as:

- ground velocity;
- planar curvature;
- course angle;
- course-rate;
- distance between the vehicle and the virtual target point.

The exact set depends on whether `Vel` or `Dist` control is selected. fileciteturn20file0

---

## Obstacle-Avoidance Extension

The repository also contains an exploratory extension toward **vector-field obstacle avoidance** in:

`MATLAB/Misc/mod_matrix_for_Obs_avoidance.m`

This component is separate from the core singularity-free path-following simulation and should be regarded as an experimental architecture.

The script combines several ideas:

- an implicit obstacle representation;
- a local normal/tangent frame;
- a first-order Euclidean distance approximation;
- a safety margin;
- a modulation matrix that attenuates the normal component near the obstacle;
- a tangential component that remains available for navigation;
- a symmetry-breaking rotational term intended to create a convergent wake behind the obstacle;
- smoothly decaying influence away from the obstacle.

For the demonstration, the obstacle is represented as an ellipse. The script computes the gradient of the implicit obstacle function, estimates local distance, constructs the modulation basis, rotates the path-following velocity, and finally transforms the result using the modulation matrix. fileciteturn9file0

This part is particularly relevant for future integration of **simultaneous path following and collision avoidance** into the singularity-free framework.

It is intentionally kept under `Misc/` because it is an extension / research direction rather than a fully integrated component of the main navigation loop.

---

## Repository Structure

```text
CV-BTP_Summer_Intern/
└── Singularity Free VF Based Path Following/
    ├── MATLAB/
    │   ├── main.m
    │   ├── main.asv
    │   ├── lib/
    │   │   ├── chebyRoots.m
    │   │   ├── createVFAtPointMatFunc.m
    │   │   ├── initializeCurveParameter.m
    │   │   ├── initializePlotTrajectory.m
    │   │   ├── navDistribController.m
    │   │   ├── navVelocityController.m
    │   │   ├── normLen.m
    │   │   ├── phiThetaToUnitVector.m
    │   │   ├── plotParams.m
    │   │   ├── plotVF.m
    │   │   └── progressbar.m
    │   └── Misc/
    │       ├── Bezier Curve Plotter/
    │       ├── Singularity Visualisation/
    │       └── mod_matrix_for_Obs_avoidance.m
    │
    └── Reading Material/
        ├── reference papers on singularity-free vector fields
        ├── vector-field path-following papers
        ├── obstacle-avoidance material
        ├── differential geometry / mathematical foundations
        ├── UAV guidance literature
        └── related numerical methods
```

The repository currently contains the MATLAB implementation and a substantial reading library. The repository tree includes the main simulation, reusable library functions, visualization helpers, an obstacle-avoidance experiment, and numerous reference PDFs. fileciteturn2file0

---

## Main Simulation Workflow

The high-level execution flow is:

```text
Choose parameterized path
        │
        ▼
Create symbolic path representation f1(w), f2(w)
        │
        ▼
Convert symbolic expressions to callable MATLAB functions
        │
        ▼
Choose initial vehicle position p0
        │
        ▼
Find suitable virtual parameter w0
(using Chebyshev-based root search + distance minimization)
        │
        ▼
Construct higher-dimensional VF
        │
        ▼
Evaluate VF at vehicle state [x,y,w]
        │
        ▼
Normalize desired VF direction
        │
        ▼
Wind-aware / curvature-aware navigation controller
        │
        ▼
Update course and vehicle velocity
        │
        ▼
Propagate physical + virtual state
        │
        ├───────────────┐
        ▼               ▼
2D projection      3D augmented trajectory
        │               │
        └───────┬───────┘
                ▼
        Post-processing / diagnostics
```

The implemented loop uses a small integration step (`dt = 0.001 s` in the current configuration) and runs for `150 s` in the active setup. fileciteturn4file0

---

## State Representation

The main simulation stores the following state vector:

```text
[x, y, w,
 dx, dy, dw,
 ddx, ddy, ddw,
 Vg,
 Kxy,
 r]
```

where:

- `x, y` = physical position;
- `w` = virtual curve parameter / augmented coordinate;
- `dx, dy, dw` = physical and virtual velocities;
- `ddx, ddy, ddw` = estimated accelerations from the discrete update;
- `Vg` = ground velocity magnitude;
- `Kxy` = planar curvature;
- `r` = distance from the current vehicle position to the virtual target point.

The auxiliary state stores angular quantities including the course angle and course-rate.

This explicit logging structure makes the simulation useful not only for trajectory visualization but also for quantitative analysis of controller behavior and feasibility constraints. fileciteturn4file0turn20file0

---

## Current Default Configuration

The current `main.m` configuration uses:

| Parameter | Current value | Meaning |
|---|---:|---|
| `dt` | `0.001 s` | Simulation timestep |
| `simTime` | `150 s` | Simulation duration |
| Path | 8-shaped | Active path example |
| `beta` | `0.01` | Parameter scaling |
| `a` | `200` | Path scale |
| `kappa` | `[0.1, 0.1]` | VF convergence shaping |
| Controller | `Dist` | Course-distribution controller |
| Airspeed `va` | `15 m/s` | Vehicle airspeed |
| Wind | `[3,4] m/s` | Constant planar wind |
| `g` | `9.81 m/s²` | Gravity |
| `phiMax` | `pi/4` | Maximum bank angle |
| `kchi` | `10 1/s` | Course correction gain |
| Initial position | `[200,-200]` | Physical initial state |

These values come directly from the current simulation script and are therefore configuration-specific rather than universal recommendations. fileciteturn4file0

---

## Important Design Choices

### Smooth nonlinear convergence functions

The implementation of the vector field uses `tanh`/`sech` terms rather than an unbounded linear error term everywhere. This gives a smoother and bounded nonlinear convergence behavior and is particularly useful when the vehicle is far from the desired path. fileciteturn5file0

### State-dependent ground speed

With wind present, ground speed depends on course angle. The simulation therefore does not simply assume

$$
V_g=V_a.
$$

Instead, it computes the physically consistent ground velocity for the prescribed airspeed and wind vector. fileciteturn4file0

### Realizability before ideality

The project explicitly constrains course-rate/curvature rather than allowing the mathematical vector field to demand arbitrarily aggressive motion. This reflects the intended application to fixed-wing-like kinematics.

### Separation of concerns

The repository separates:

- field generation;
- parameter initialization;
- vehicle/controller realization;
- visualization;
- post-processing;
- experimental extensions.

This makes the framework easier to modify for other path definitions and control architectures.

---

## Additional Utilities

### `normLen.m`

This helper solves for a point on an implicit curve that satisfies the normality condition with respect to a given point. It uses `fsolve` on the curve equation together with the orthogonality condition between the displacement vector and curve tangent. fileciteturn15file0

This is useful for experiments involving nearest-point / normal-distance calculations, although it is not the central mechanism of the current augmented path-following loop.

### `phiThetaToUnitVector.m`

Converts spherical-angle coordinates `(phi, theta)` into a 3D unit vector, primarily for visualization / plane definitions. fileciteturn16file0

### Bezier curve plotter

The miscellaneous directory contains an HTML-based parametric curve plotting utility intended to help generate or inspect parameterized curves before using them in the vector-field framework.

---

## Relation to the Reference Paper

The project is grounded in the paper's main theoretical pipeline:

**physical path**

→ parameterization

→ **higher-dimensional path**

→ **singularity-free VF**

→ extended / transformed dynamics

→ projection back to physical coordinates.

The reference paper additionally gives rigorous results for:

- the impossibility of global convergence in the conventional time-invariant same-dimensional case;
- elimination of singular points by dimensional extension;
- global asymptotic convergence of the projected physical trajectory;
- local/global exponential convergence of the path-following error under bounded derivatives;
- robustness to bounded disturbances through input-to-state-stability arguments;
- interpretation of the additional coordinate as a state-dependent time-like variable. fileciteturn0file0

The project's MATLAB code should therefore be understood as an **engineering/simulation realization of the core theoretical construction**, together with additional numerical and vehicle-modeling layers.

---

## Improvements and Extensions Explored in This Project

The repository contains several elements that make it more than a minimal reproduction of the theoretical construction.

### 1. Practical parameter initialization

The reference formulation assumes a parameterized curve, but a practical navigation system also needs a sensible initial phase/parameter. The project implements an explicit search procedure to initialize `w0` using stationary points of distance and numerical root finding. fileciteturn12file0

### 2. Numerical robustness for nonlinear functions

The custom Chebyshev root solver extends the numerical toolkit beyond simple polynomial cases and incorporates adaptive degree refinement, interval splitting, false-convergence checking, and Newton/Brent-style numerical refinement through `fzero`. fileciteturn10file0

### 3. Wind-aware fixed-wing-inspired navigation

Instead of treating the desired VF vector as directly achievable, the project accounts for wind and uses a course-based controller with a physical turning constraint. fileciteturn7file0turn4file0

### 4. Two navigation-control architectures

Both velocity-vector and course-distribution approaches are retained, making it possible to compare how the same vector field behaves when coupled to different low-level motion models. fileciteturn17file0turn7file0

### 5. Obstacle-avoidance direction

The obstacle-modulation script explores how the singularity-free VF concept may be combined with local field deformation and safety buffers. This is not yet a single integrated planner, but it establishes a path toward combined **path following + collision avoidance**. fileciteturn9file0

### 6. Diagnostics focused on physical feasibility

The post-processing layer explicitly examines course-rate, curvature, ground velocity, and virtual-target distance, making the results more informative than a trajectory-only plot. fileciteturn20file0

---

## How to Run

### Requirements

The implementation is written for MATLAB and uses functionality from the Symbolic Math / Optimization toolchains available in standard MATLAB environments.

You should have access to:

- MATLAB;
- Symbolic Math Toolbox for symbolic path construction and differentiation;
- Optimization Toolbox for `fsolve` where the corresponding utility is used.

### Basic execution

1. Clone the repository.
2. Open MATLAB.
3. Set the working directory to:

```text
Singularity Free VF Based Path Following/MATLAB
```

4. Open `main.m`.
5. Select or define a parameterized path.
6. Adjust simulation and controller parameters as required.
7. Run:

```matlab
main
```

The script automatically adds the `lib` directory and initializes the simulation. fileciteturn4file0

### Defining a new path

The simplest workflow is to define a symbolic pair:

```matlab
syms w
fnS.x = ...;   % x = f1(w)
fnS.y = ...;   % y = f2(w)
```

Then select gains and a suitable simulation region.

The rest of the framework can use the same parameterized representation to:

- construct the higher-dimensional VF;
- initialize `w0`;
- simulate the trajectory;
- visualize the physical and augmented path.

---

## Interpreting the Plots

### Projected 2D path-following plot

This plot shows the actual physical trajectory relative to the desired path. For self-intersected curves, this is the most intuitive view of whether the controller can traverse the path without getting trapped at a crossing.

### Augmented 3D path-following plot

This plot shows `(x,y,w)`. A self-intersection in the physical plane can separate cleanly along the `w` axis, making it visually clear why the higher-dimensional representation eliminates the ambiguity at a crossing.

### Course-angle plot

The course angle should change continuously enough to remain within the vehicle's turning capabilities.

### Course-rate and curvature plots

These are feasibility diagnostics. The desired curvature should remain inside the bounds imposed by the assumed maximum bank angle / turning capability.

### Virtual-target distance

The distance from the vehicle to the virtual point is an indicator of how aggressively the augmented path representation is using its state-dependent parameter to reduce the physical path error.

---

## Known Limitations

This repository should be considered a research implementation rather than a production-ready flight-control stack.

### 1. The current main simulator is not a full six-degree-of-freedom aircraft model

The navigation layer is represented using a simplified planar/course-based kinematic model with vertical/augmented components rather than complete rigid-body aircraft dynamics.

### 2. Numerical tuning is important

The choice of convergence gains, parameter scaling, timestep, wind, and curvature limit can materially affect behavior.

### 3. Virtual-coordinate sensitivity

Because `w` participates directly in the augmented vector field, poor curve parameterization can lead to undesirable sensitivity. The reference paper discusses re-parameterization and scaling as practical remedies. fileciteturn0file0

### 4. The obstacle-avoidance extension is exploratory

`mod_matrix_for_Obs_avoidance.m` demonstrates a promising field-modulation architecture, but it is not yet integrated into the main closed-loop path-following simulation or accompanied here by the same level of theoretical guarantees as the core singularity-free construction. fileciteturn9file0

### 5. Constant-speed behavior is not automatic in the augmented formulation

The virtual coordinate contributes an additional component to the higher-dimensional dynamics. The reference paper explicitly identifies constant-speed operation as a non-trivial issue and discusses partial normalization as a future direction. fileciteturn0file0

---

## Research Questions Explored

The repository can be used to study several useful questions:

1. How does dimensional extension eliminate singularities caused by self-intersections?
2. How does the choice of curve parameterization affect the conditioning and smoothness of the vector field?
3. What is the trade-off between convergence gain and realizable vehicle curvature?
4. How does constant wind alter the relationship between airspeed, course angle, and ground velocity?
5. How should the initial virtual coordinate be selected for a general parameterized path?
6. Can obstacle modulation be combined with the singularity-free path field without creating new undesirable equilibria?
7. Can the method be extended to 3D physical paths and higher-dimensional configuration spaces?
8. How can the parameterization be optimized for minimum tracking error, minimum control effort, or maximum robustness?

---

## Suggested Experimental Methodology

For a systematic evaluation, the following progression is recommended:

### Experiment 1 — Simple closed path

Start with a circle or ellipse and compare:

- conventional 2D VF;
- augmented 3D VF.

Visualize the singularity of the conventional field and the absence of a zero field in the augmented representation.

### Experiment 2 — Self-intersected path

Use the current 8-shaped curve and initialize the vehicle from multiple locations around the crossing. Measure whether trajectories converge to the same geometric path without becoming trapped at the intersection.

### Experiment 3 — Parameterization study

Keep the physical geometry fixed while changing

$$
w \mapsto g(w)
$$

and examine the resulting field smoothness, `w` evolution, curvature demand, and convergence rate.

### Experiment 4 — Wind sensitivity

Sweep the wind vector and compare:

- ground velocity;
- course-rate;
- path error;
- curvature margin.

### Experiment 5 — Controller comparison

Run the same path with both:

```matlab
params.controller = "Vel";
```

and

```matlab
params.controller = "Dist";
```

and compare the resulting trajectory and feasibility metrics.

### Experiment 6 — Obstacle avoidance

Use the modulation experiment as a starting point for a coupled path-following / collision-avoidance controller and investigate whether the resulting field preserves convergence away from obstacles while maintaining the safety margin.

---

## Reading Material

The repository contains a broad collection of material used to understand the project, including papers on:

- guiding vector fields for nonholonomic mobile robots;
- singularity-free guiding vector fields;
- distributed coordinated path following;
- UAV/fixed-wing guidance;
- vector-field obstacle avoidance;
- differential geometry of curves and surfaces;
- Chebyshev approximation and numerical root finding.

The reading material is intentionally broader than the papers directly implemented in the code. It serves as a research reference base for understanding the mathematical formulation, alternative VF designs, vehicle guidance methods, obstacle avoidance, and numerical tools.

The repository tree confirms that these papers and mathematical references are stored alongside the code. fileciteturn3file0

The attached paper is the primary conceptual reference for the singularity-free higher-dimensional construction. fileciteturn0file0

---

## Technical Summary

| Layer | Main idea | Repository implementation |
|---|---|---|
| Path representation | Parameterized geometric curve | `fnS.x`, `fnS.y` in `main.m` |
| Dimensional extension | Add virtual coordinate `w` | `[x,y,w]` state |
| Path constraints | `phi_i = x_i - f_i(w)` | `createVFAtPointMatFunc.m` |
| Guiding field | Tangential + converging field | generated in `createVFAtPointMatFunc.m` |
| Desired direction | Normalize VF | `main.m` |
| Initial parameter | Closest stationary point | `initializeCurveParameter.m` |
| Root finding | Chebyshev approximation + matrix roots | `chebyRoots.m` |
| Vehicle realization | Velocity-vector control | `navVelocityController.m` |
| Vehicle realization | Course-distribution control | `navDistribController.m` |
| Physical constraint | Course-rate / curvature limiting | controller + `plotParams.m` |
| Wind | Ground-speed correction | `main.m` / `navDistribController.m` |
| Visualization | 2D + 3D field/trajectory views | `plotVF.m`, `initializePlotTrajectory.m` |
| Diagnostics | Ground speed, course, curvature, distance | `plotParams.m` |
| Collision avoidance | Modulation matrix + wake shaping | `Misc/mod_matrix_for_Obs_avoidance.m` |

---

## Conceptual Takeaway

The most important idea in this project is not a particular MATLAB function. It is the change in viewpoint:

> **Instead of trying to force a time-invariant vector field to globally navigate a difficult path in the original space, augment the state space so that the path becomes topologically well behaved, construct the field there, and then project the resulting motion back onto the physical space.**

For a self-intersecting 2D path, the crossing is ambiguous in the physical plane because multiple path branches meet at the same `(x,y)` point. The additional coordinate `w` separates those branches into different locations in the augmented space.

This turns a geometric ambiguity into a state-space distinction.

That is the central mathematical insight inherited from the reference work, while the repository develops the surrounding numerical and control machinery needed to explore how such a field behaves in a realistic simulation setting. fileciteturn0file0

---

## Future Work

The natural next steps for the project include:

- integrate obstacle avoidance directly into the main singularity-free VF loop;
- investigate formally guaranteed safety under modulation;
- extend from planar physical paths to general 3D parameterized paths;
- compare different smooth error functions (`linear`, `tanh`, other bounded nonlinearities);
- study optimal parameterizations for minimizing tracking error and control effort;
- investigate partial normalization for improved constant-speed behavior;
- add more realistic fixed-wing dynamics and actuator limits;
- quantify robustness under sensor noise and time-varying wind;
- evaluate the method over large families of self-intersecting paths;
- benchmark computational cost of the augmented VF against conventional VF-PF and trajectory-tracking approaches.

These directions align closely with open problems discussed in the reference paper, particularly constant-speed operation, collision avoidance, and optimal parameterization. fileciteturn0file0

---

## Citation

### Primary reference

W. Yao, H. Garcia de Marina, B. Lin, and M. Cao, **“Singularity-free Guiding Vector Field for Robot Navigation,”** arXiv:2012.01826, version 3, 23 Oct. 2021. The paper develops the impossibility result for same-dimensional time-invariant VF-PF, the higher-dimensional transformation, the singularity-free construction, and the global convergence guarantees. fileciteturn0file0

### Closely related work

The reading material also includes foundational and related work on guiding vector fields, 3D path following, fixed-wing guidance, obstacle avoidance, distributed path following, and differential geometry. These sources are retained in the repository for research context rather than implying that every included reference is implemented directly.

---

## Project Status

**Status:** Research / simulation implementation

**Primary language:** MATLAB

**Main focus:** Singularity-free higher-dimensional vector-field path following

**Current emphasis:** Parameterized 2D paths embedded into 3D augmented state space, with wind-aware course control and curvature constraints

**Extensions:** Numerical root finding, visualization, and exploratory obstacle avoidance

---

## Repository

[GitHub Repository](https://github.com/dhe7871/CV-BTP_Summer_Intern)

