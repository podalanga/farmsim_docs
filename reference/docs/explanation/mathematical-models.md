# Mathematical Models — CPG ODEs, hydrodynamics, and Ekeberg torque

!!! note "Source Files"
    - `farms_amphibious/farms_amphibious/control/ode.pyx` — CPG oscillator ODE integration
    - `farms_amphibious/farms_amphibious/control/ekeberg.pyx` — Ekeberg muscle model
    - `farms_mujoco/farms_mujoco/swimming/drag.pyx` — drag force/torque computation (`compute_drag_force`, `compute_drag_torque`, `compute_link_drag_fast`)
    - `farms_mujoco/farms_mujoco/swimming/buoyancy_cy.pyx` — buoyancy force/torque computation (`compute_buoyancy_analytic_fast`, `compute_link_buoyancy_fast`)
    - `farms_mujoco/farms_mujoco/swimming/hydrodynamics.pyx` — orchestration layer (`SwimmingHandler`, `WaterProperties*`, `link_swimming_info`, `compute_link_forces`, `apply_swimming_forces`) that combines drag + buoyancy per link and writes into the xfrc sensor array

## 1. CPG Oscillator Equations & Coupling Terms
Implemented in `farms_amphibious/control/ode.pyx`.

**Phase ODE:**

$$
\dot{\theta}_i = \omega_i \left[ 1 + A_{mod, i} \cos(\theta_i + \phi_{mod, i}) \right] + \sum_j A_j w_{ij} \sin(\theta_j - \theta_i - \phi_{bias, ij}) + \mathcal{S}_{\theta, i}
$$

**Amplitude ODE:**

$$
\dot{A}_i = r_{A, i} (A_{nom, i} - A_i) + \mathcal{S}_{A, i}
$$

**Joint Offset ODE:**

$$
\dot{x}_{off, i} = r_{off, i} (x_{off\_des, i} - x_{off, i})
$$

Where:

- $\theta_i$: Phase of oscillator $i$ (rad)

- $\omega_i$: Intrinsic angular frequency of oscillator $i$ (rad/s)

- $A_{mod, i}$: Modular amplitude (frequency modulation depth)

- $\phi_{mod, i}$: Modular phase offset

- $A_i$: Amplitude of oscillator $i$

- $w_{ij}$: Coupling weight from oscillator $j$ to oscillator $i$

- $\phi_{bias, ij}$: Desired phase difference between $j$ and $i$

- $r_{A, i}$: Convergence rate for amplitude (1/s)

- $A_{nom, i}$: Nominal (target) amplitude

- $x_{off, i}$: Joint offset position

- $r_{off, i}$: Convergence rate for joint offset (1/s)

- $x_{off\_des, i}$: Desired joint offset position

- $\mathcal{S}_{\theta, i}$, $\mathcal{S}_{A, i}$: Sensory feedback terms for phase and amplitude

**Sensory Feedback Terms ($\mathcal{S}_{\theta, i}$ and $\mathcal{S}_{A, i}$):**

- **Stretch to Phase (Direct):** $w \cdot \theta_{joint}$

- **Stretch to Phase (Tegotae):** $w \cdot \theta_{joint} \cdot \sin(\theta_i)$

- **Contact Reaction to Phase:** $w \cdot |F_{react}|$

- **Lateral Force to Phase/Amplitude:** $w \cdot |F_{y}|$

Where:

- $w$: Feedback coupling weight

- $\theta_{joint}$: Measured joint position error

- $F_{react}$: Ground reaction force

- $F_{y}$: Lateral hydrodynamic force

**Ekeberg Muscle Model (Output Coupling)**
Implemented in `farms_amphibious/control/ekeberg.pyx`.
Total Torque:

$$
\tau = \tau_{act} + \tau_{act\_stiff} + \tau_{pass\_stiff} + \tau_{damp} + \tau_{fric}
$$

Where:

- $\tau_{act} = \alpha (M_L - M_R)$: Active torque, which is proportional to the difference between flexor ($M_L$) and extensor ($M_R$) activations (the net pull).

- $\tau_{act\_stiff} = \beta (M_L + M_R)(\phi_{off} - \phi_{joint})$: Active stiffness, representing the spring-like resistance caused by muscle co-contraction (the sum of activations).

- $\tau_{pass\_stiff} = \gamma \beta (\phi_{off} - \phi_{joint})$: Passive stiffness, acting as a baseline structural spring returning the joint to equilibrium.

- $\tau_{damp} = -\delta \dot{\phi}_{joint}$: Viscous damping, providing resistance proportional to the joint's velocity.

- $\tau_{fric} = -\epsilon \operatorname{sgn}(\dot{\phi}_{joint})$: Coulomb friction, providing constant resistance opposing the direction of motion.

Definitions:

- $M_L, M_R$: Neural output activations for left/flexor and right/extensor muscles

- $\alpha$: Active torque gain

- $\beta$: Stiffness coefficient

- $\gamma$: Passive stiffness ratio

- $\delta$: Viscous damping coefficient

- $\epsilon$: Coulomb friction coefficient

- $\phi_{off}$: Reference joint equilibrium position

- $\phi_{joint}$: Current joint position

- $\dot{\phi}_{joint}$: Current joint velocity

## 2. Hydrodynamic Force Models
Implemented across `farms_mujoco/swimming/drag.pyx` (drag), `farms_mujoco/swimming/buoyancy_cy.pyx` (buoyancy), and `farms_mujoco/swimming/hydrodynamics.pyx` (orchestration: `SwimmingHandler` drives `compute_link_forces`/`apply_swimming_forces` per link per step).

!!! warning "DISCREPANCY FLAG"
    The documentation mentions "added mass, drag, lift". However, **added mass and lift are completely absent from the codebase.** The implementation only handles drag (`drag.pyx`) and buoyancy (`buoyancy_cy.pyx`), combined per link by `hydrodynamics.pyx`.

**Translational Drag Force (in URDF frame):**

$$
F_{drag, i} = C_{d, i} \cdot \mu \cdot v_{rel, i} |v_{rel, i}| + F_{buoyancy, i}
$$

*(Note on sign convention: The code computes $v|v|$. For this force to act as drag opposing motion, the configuration coefficients $C_d$ must be defined as negative values. If they are positive, this equation acts as thrust.)*

**Rotational Drag Torque (in URDF frame, not CoM frame):**

$$
\tau_{drag, i} = C_{\tau, i} \cdot \omega_{rel, i} |\omega_{rel, i}|
$$

Unlike the translational force, torque has **no** $\mu$ (viscosity) factor —
`compute_drag_torque` (`drag.pyx`) multiplies only by the coefficient, not
by viscosity. This is easy to miss since the two functions look almost
identical.

Both $v_{rel,i}$ and $\omega_{rel,i}$ are computed in the link's **URDF
frame** (the same frame the linear/angular velocity are rotated into by
`link_swimming_info` via `global2urdf` — see §4 below), not the CoM frame.

## 2a. Buoyancy: two independent methods, selected by `cob_method`

`arena.water.cob` (or the flat `cob_*` fields — see `cob_options.py`)
selects one of three `cob_method` values, resolved once at
`SwimmingHandler.__init__` time into the `use_exact_cob`/`force_mesh` bints
`compute_link_forces` branches on every link every step:

| `cob_method` | Function used | What it computes |
|---|---|---|
| `'ramp'` | `compute_buoyancy_analytic_fast` (`buoyancy_cy.pyx`) | Single-point bounding-sphere linear ramp — O(1), approximate |
| `'analytic'` | `compute_link_buoyancy` → `compute_buoyancy_mesh`, using each primitive's closed-form solution when available (currently spheres, `submerged_sphere_analytic_cy`) and mesh-clip otherwise | Exact submerged volume/centroid per primitive |
| `'mesh'` | Same as `'analytic'` but `force_mesh=True` forces every primitive through the mesh-clip path, even spheres | Exact, no closed-form shortcuts (used to validate the analytic path) |

### Ramp method

$$
F_{buoyancy, z} = -\rho_{water} \cdot \frac{m_{link} \cdot g}{\rho_{link}} \cdot \min\left( \frac{z_{surf} + r_{link} - z_{link}}{2 r_{link}}, 1 \right)
$$

applied only when $z_{link} - r_{link} < z_{surf}$ and $m_{link} > 0$;
otherwise $F_{buoyancy} = 0$.

!!! warning "Water density is a parameter, not a constant"
    $\rho_{water}$ is `water_density` — sourced from `arena.water.density`
    in the YAML config (or a spatially-varying callback if water maps are
    used), **not** a hardcoded `1000`. `1000` only appears in the codebase
    as `WaterProperties`'s *default*, used before a real value overrides it.

!!! note "The sign works out because `gravity` is passed as `-9.81`"
    `SwimmingHandler.step()` (`hydrodynamics.pyx`) hardcodes
    `gravity=-9.81` when calling into buoyancy/drag (see §4 in the
    hydrodynamics internals for why this is not configurable). The actual
    code line is `buoyancy[2] = -water_density * mass * gravity / density *
    min(...)` — substituting `gravity = -9.81` flips the sign, giving a
    net **positive** (upward) force, which is what's shown above with
    `g = +9.81` and the leading `-ρ` already accounting for it.

$r_{link}$ (`bound_radius`) is the link's bounding-sphere radius
(`physics.model.geom_rbound` — already a radius, not a diameter — from a
geom in collision group 2), not a half-height.

### Exact (analytic / mesh) methods

For each of a link's collision primitives, `submerged_volume_and_centroid_fast`
computes the exact submerged volume $V_j$ and centroid $\vec{c}_j$ (either
via the sphere closed form or a per-triangle tetrahedron-decomposition clip
against the water plane — see
[Hydrodynamics Internals](../internals/hydrodynamics-internals.md) for the
`buoyancy.py`/`buoyancy_cy.pyx` split). These are volume-weighted and
combined into a single center of buoyancy for the link:

$$
V = \sum_j V_j \qquad \vec{c}_{CoB} = \frac{\sum_j V_j \vec{c}_j}{V}
$$

$$
\vec{F}_{buoyancy} = \begin{bmatrix} 0 \\ 0 \\ -\rho_{water} \, g \, V \end{bmatrix}
\qquad
\vec{\tau}_{buoyancy} = (\vec{c}_{CoB} - \vec{c}_{CoM}) \times \vec{F}_{buoyancy}
$$

both computed in the **world frame** then rotated back to the URDF frame
(`R_global2urdf @ ...`) before returning to `compute_link_forces`. Unlike
the ramp method, this produces a genuine righting/capsizing torque whenever
the center of buoyancy and center of mass don't coincide — the ramp method
always returns zero buoyancy torque (`buoyancy_torque[i] = 0` for all `i`
in `compute_link_buoyancy_fast`'s ramp branch).

If an exact method is requested but a link has no usable collision
primitives (e.g. all geoms are of an unsupported type), it silently falls
back to the ramp formula above for that link only
(`compute_link_buoyancy`'s `if primitives: ... else: compute_buoyancy_analytic(...)`).

## 3. Integration Schemes
Implemented in `farms_amphibious/control/network.py` via `scipy.integrate.ode`.

- **Scheme:** `dopri5`
- **Method:** Dormand-Prince Runge-Kutta 4(5).
- **Details:** Explicit Runge-Kutta method with adaptive step sizing, chosen for its stability in integrating stiff non-linear equations like coupled CPG oscillators. Bounded in the integration loop to align with the physics time step.

## 4. Coordinate Frame Transforms
Implemented in `farms_mujoco/swimming/hydrodynamics.pyx` (`link_swimming_info`). Uses quaternions (`quat_conj`, `quat_mult`, `quat_rot` from `farms_core.utils.transform`) for transforming vectors.

**Orientations computed by `link_swimming_info` every link, every step:**

$$
q_{urdf \to global}, \ q_{com \to global} \quad \text{(read directly from sensor data)}
$$

$$
q_{global \to urdf} = q_{urdf \to global}^* \qquad\text{(used downstream — see below)}
$$

$$
q_{com \to urdf} = q_{global \to urdf} \cdot q_{com \to global}, \qquad q_{urdf \to com} = q_{com \to urdf}^* \qquad\text{(computed, but see warning below)}
$$

**Vector Rotations actually used in the hydrodynamics hot path:**

$$
v_{urdf} = q_{global \to urdf} \cdot v_{global} \cdot q_{global \to urdf}^*
$$

$$
F_{global,\ final} = q_{urdf \to global} \cdot F_{urdf} \cdot q_{urdf \to global}^*
$$

Forces and torques stay in the **URDF frame** through the entire drag +
buoyancy computation (`compute_drag_force`, `compute_drag_torque`,
`compute_buoyancy_analytic_fast`) and are rotated **directly back to world
frame** at the very end of `compute_link_forces`, via `urdf2global` — never
via a CoM-frame intermediate.

!!! warning "`q_com→urdf` / `q_urdf→com` are computed but never used for forces"
    `link_swimming_info` also computes `com2global`, `com2urdf`
    (`q_{global \to urdf} \cdot q_{com \to global}`), and `urdf2com` (its
    conjugate) every call — but nothing downstream reads them.
    `compute_link_forces` only ever passes `global2urdf`/`urdf2global`
    onward to the drag/buoyancy functions. There is no `F_com = q_urdf→com
    · F_urdf · q_urdf→com*` rotation anywhere in the codebase — an earlier
    version of this page implied one existed; it doesn't. The only use of
    the CoM frame in the exact-buoyancy path is reading
    `com_position_cy()` directly (which already returns world-frame
    coordinates, not a frame to rotate through) for the CoB-to-CoM torque
    arm. See [Hydrodynamics Internals](../internals/hydrodynamics-internals.md#6-dead-computation-com2urdfurdf2comcom2global-are-computed-but-never-used)
    for the full trace.

Where:

- $q_{global \to urdf}$: Quaternion representing rotation from the global world frame to the URDF link frame

- $q_{urdf \to global}^*$: Complex conjugate (inverse) of the rotation from URDF to global frame

- $v_{global}$: Velocity vector in the global coordinate frame

- $v_{urdf}$: Velocity vector rotated into the local URDF coordinate frame

- $F_{urdf}$: Combined drag+buoyancy force vector in the local URDF frame

- $F_{global,\ final}$: Same force, rotated to world frame — this is what's
  written into `data_xfrc.array`, and is already what `xfrc_applied`
  expects (see the hydrodynamics internals page for the separate,
  confirmed bug where `SwimmingExtension.before_step` rotates this a
  second time)
