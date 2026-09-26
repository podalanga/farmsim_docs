# Mathematical Models: CPG, muscles and hydrodynamics

!!! note "Source Files"
    - `farms_amphibious/farms_amphibious/control/ode.pyx`: CPG oscillator ODE integration
    - `farms_amphibious/farms_amphibious/control/ekeberg.pyx`: Ekeberg muscle model
    - `farms_mujoco/farms_mujoco/swimming/cob.pyx`: submerged volume and centre of buoyancy
    - `farms_mujoco/farms_mujoco/swimming/drag.pyx`: legacy quadratic drag
    - `farms_mujoco/farms_mujoco/swimming/ellipsoid_model.pyx`: ellipsoid drag and added mass
    - `farms_mujoco/farms_mujoco/swimming/hydrodynamics.pyx`: `SwimmingHandler`, which combines them per link

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

- $\theta_{joint}$: Measured joint position

- $F_{react}$: Ground reaction force

- $F_{y}$: Lateral external force of a link (`xfrc` sensors, which include the fluid forces)

### Joint equations

The oscillator outputs are $M_i = A_i (1 + \cos\theta_i)$. Each joint is
driven by two oscillators, $M_1$ and $M_2$ (`osc1` and `osc2` of the
joint's entry in `control.muscles`, usually the left and the right one).
The motor `equation` decides how they become a joint command. $k$ and $b$
are the motor's `transform.gain` and `transform.bias`, which convert from
the network convention to the joint of the model, and $\phi_{off}$ is the
joint offset of the offset ODE above.

**`position_muscle`** (`farms_amphibious/control/position_muscle_cy.pyx`),
a position command:

$$
\phi_{cmd} = k \left( \tfrac{1}{2}(M_2 - M_1) + \phi_{off} \right) + b
$$

**`ekeberg_muscle` / `ekeberg_muscle_explicit`**
(`farms_amphibious/control/ekeberg.pyx`), a torque:

$$
\tau = \underbrace{\alpha (M_2 - M_1)\, k}_{\text{active}}
+ \underbrace{\beta (M_1 + M_2)\, \Delta\phi\, k}_{\text{active stiffness}}
+ \underbrace{\gamma \beta\, \Delta\phi\, k}_{\text{passive stiffness}}
\underbrace{- \delta\, \dot\phi}_{\text{damping}}
\underbrace{- \epsilon \operatorname{sgn}(\dot\phi)}_{\text{friction}}
$$

with $\Delta\phi = \phi_{off} - \frac{\phi - b}{k}$ the distance to the
offset in the network convention, $\phi$ and $\dot\phi$ the joint position
and velocity, and $\alpha$, $\beta$, $\gamma$, $\delta$, $\epsilon$ the
coefficients of `control.muscles`. With $k = 1$ and $b = 0$, this is the
usual Ekeberg model. The active term follows the difference of the two
activations, the stiffness terms the co-contraction. In
`ekeberg_muscle`, the stiffness and damping are applied through the MuJoCo
joint (spring reference, stiffness and damping), and in
`ekeberg_muscle_explicit` as a torque.

## 2. Hydrodynamics

The fluid wrench of each link with `fluid_interaction: true` is computed
by `SwimmingHandler` (`farms_mujoco/swimming/hydrodynamics.pyx`) at every
environment step, in the world frame and about the link centre of mass
(CoM). It is the sum of the terms below. The implementation is described
in [Hydrodynamics Internals](../internals/hydrodynamics-internals.md).

### 2.1 Buoyancy

With $V$ the submerged volume of the link and $c_b$ its centroid (the
centre of buoyancy), $\rho$ the water density and $g$ the gravity vector
of the MuJoCo model:

$$
F_b = -\rho V g, \qquad \tau_b = (c_b - c_m) \times F_b
$$

where $c_m$ is the CoM. The torque is what rights (or capsizes) a floating
body. $V$ and $c_b$ come from the link geoms (collision geoms by default,
`cob_geom_group`), with `cob_method`:

- `exact`: exact volume and first moment of each geom below the water
  plane, summed over the geoms: $V = \sum_j V_j$,
  $c_b = \frac{1}{V}\sum_j V_j c_j$. Spheres, ellipsoids, cylinders and
  capsules are computed in closed form (or to 1e-10 for capsules), boxes
  and meshes with the divergence theorem.
- `lut`: interpolated from a per-link table of $V$ and $V c_b$ for all
  water directions and depths (about 1 % of the link volume).
- `ramp` (legacy): $V = \frac{m}{\rho_{\text{link}}} \min\left(\frac{h + r - z}{2r}, 1\right)$,
  with $m$ the link mass, $\rho_{\text{link}}$ its `density`, $r$ its
  bounding radius, $z$ its height and $h$ the surface height. The force
  is applied at the CoM, without torque.

### 2.2 Legacy drag (`fluid_model: legacy`)

In the link frame, with $v$ and $\omega$ the linear and angular velocities
of the link relative to the water, $\mu$ the water `viscosity` option and
$c$, $c'$ the two rows of the link's `drag_coefficients`:

$$
F_i = \mu\, c_i\, v_i |v_i|, \qquad \tau_i = c'_i\, \omega_i |\omega_i|
$$

The coefficients are negative so that the drag opposes the motion.
Different coefficients along the body axis and across it give the
anisotropic drag that makes undulatory swimming possible. The torque has
no $\mu$ factor. With `drag_implicit: true`, the force uses the velocity
$v'$ at the end of the step, solution of
$m (v' - v)/\Delta t = \mu c_i v' |v'| + f$ (backward Euler), which is
stable for any timestep.

### 2.3 Ellipsoid model (`fluid_model: ellipsoid`)

Each link is approximated by an ellipsoid of semi-axes $r = (a, b, c)$,
either the minimum volume ellipsoid enclosing its geoms (`mvee`) or the
ellipsoid with its inertia (`inertia`). In the ellipsoid frame, with
$C_{\text{form}}$, $C_{\text{visc}}$ and $C_{\text{rot}}$ the
`ellipsoid_coefficients` and $\mu_d$ the `dynamic_viscosity`:

$$
F_{\text{form}} = -\tfrac{1}{2} \rho\, C_{\text{form}}\, A(v)\, |v|\, v,
\qquad
A(u) = \pi \sqrt{(bc\,u_x)^2 + (ac\,u_y)^2 + (ab\,u_z)^2}
$$

($A$ is the area of the ellipsoid projected along the unit vector $u$ of
the velocity),

$$
F_{\text{visc}} = -6\pi \mu_d\, r_D\, C_{\text{visc}}\, v,
\qquad
\tau_{\text{visc}} = -8\pi \mu_d\, r_D^3\, C_{\text{visc}}\, \omega
$$

with $r_D$ the radius of the sphere of equal volume, and

$$
\tau_{\text{rot}, i} = -\rho\, C_{\text{rot}}\, I_{D,i}\, \omega_i |\omega|,
\qquad
I_{D,i} = \tfrac{8\pi}{15}\, r_i \max(r_j, r_k)^4
$$

The quadratic terms are scaled by the water `viscosity` option (1 by
default), and every term by the submerged fraction $V/V_{\text{full}}$.

### 2.4 Added mass (`added_mass`, ellipsoid model)

A body accelerating in a fluid also accelerates fluid around it. For an
ellipsoid, Lamb's coefficients give the added mass $m_A$ and inertia
$I_A$ along the axes:

$$
\kappa_i = abc \int_0^\infty \frac{d\lambda}{(r_i^2 + \lambda)\sqrt{(a^2+\lambda)(b^2+\lambda)(c^2+\lambda)}},
\qquad
m_{A,i} = \rho V \frac{\kappa_i}{2 - \kappa_i}
$$

$$
I_{A,i} = \frac{\rho V}{5} \frac{(r_j^2 - r_k^2)^2 (\kappa_k - \kappa_j)}{2 (r_j^2 - r_k^2) + (r_j^2 + r_k^2)(\kappa_j - \kappa_k)}
$$

The velocity dependent (Kirchhoff) terms are always applied, including
the Munk moment that turns an elongated body across the flow:

$$
F_A = (m_A \circ v) \times \omega, \qquad
\tau_A = (m_A \circ v) \times v + (I_A \circ \omega) \times \omega
$$

The acceleration terms $-m_A \circ \dot v$ and $-I_A \circ \dot \omega$
depend on `added_mass`:

- `implicit`: the added mass (mean of $m_A$, times the submerged fraction)
  and the added inertia are added to the MuJoCo body mass and inertia, as
  in Stonefish, and their weight is cancelled. MuJoCo then integrates them
  implicitly, which is always stable.
- `explicit`: they are applied as forces, with $\dot v$ and $\dot\omega$
  from filtered finite differences. This is unstable when the added mass
  exceeds about half of the link mass.

## 3. Integration Schemes

The CPG network (phases, amplitudes and joint offsets) is integrated by
`farms_amphibious/control/network.py` with `scipy.integrate.ode` and the
`dopri5` integrator (Dormand-Prince Runge-Kutta 4(5), explicit, with
adaptive steps). At every environment step, the solver integrates up to
the end of the MuJoCo step, and the state is stored in the network state
array at the current iteration.

MuJoCo integrates the mechanics with the integrator of the simulation
file (`mujoco.integrator`, `implicitfast` for the Zbot). The fluid
forces are explicit forces for MuJoCo, except the implicit added mass,
which modifies the body masses and inertias.

## 4. Frames

The link states come from the `links` sensors (world frame): CoM position,
orientation of the link frame and CoM velocities. The fluid forces are
computed in the world frame. The legacy drag rotates the relative
velocities into the link frame with the link rotation matrix $R$
($v_{\text{link}} = R^T v$), computes the drag there and rotates it back
($F = R F_{\text{link}}$). The ellipsoid model does the same with the
ellipsoid frame. The resulting wrench, about the CoM and in the world
frame, is what MuJoCo's `xfrc_applied` expects, and it is also stored in
the `xfrc` sensors.
