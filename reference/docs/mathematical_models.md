# Mathematical Models — CPG ODEs, hydrodynamics, and Ekeberg torque

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
Implemented in `farms_mujoco/swimming/drag.pyx`.

!!! warning "DISCREPANCY FLAG"
    The documentation mentions "added mass, drag, lift". However, **added mass and lift are completely absent from the codebase.** The implementation in `drag.pyx` only handles drag and buoyancy.

**Translational Drag Force (in URDF frame):**

$$
F_{drag, i} = C_{d, i} \cdot \mu \cdot v_{rel, i} |v_{rel, i}| + F_{buoyancy, i}
$$

*(Note on sign convention: The code computes $v|v|$. For this force to act as drag opposing motion, the configuration coefficients $C_d$ must be defined as negative values. If they are positive, this equation acts as thrust.)*

**Rotational Drag Torque (in CoM frame):**

$$
\tau_{drag, i} = C_{\tau, i} \cdot \omega_{rel, i} |\omega_{rel, i}|
$$

**Buoyancy (in global frame rotated to URDF frame):**

$$
F_{buoyancy, z} = 1000 \cdot \frac{m_{link} \cdot 9.81}{\rho_{link}} \cdot \min\left( \frac{z_{surf} + h_{link} - z_{com}}{2h_{link}}, 1 \right)
$$

Where:

- $F_{drag, i}$: Translational drag force on link $i$

- $C_{d, i}$: Drag coefficient for link $i$

- $\mu$: Fluid density/viscosity factor

- $v_{rel, i}$: Relative translational velocity of the link

- $F_{buoyancy, i}$: Buoyancy force acting on link $i$

- $\tau_{drag, i}$: Rotational drag torque on link $i$

- $C_{\tau, i}$: Rotational drag coefficient

- $\omega_{rel, i}$: Relative angular velocity of the link

- $F_{buoyancy, z}$: Vertical component of the buoyancy force

- $m_{link}$: Mass of the link

- $\rho_{link}$: Density of the link

- $z_{surf}$: Z-coordinate of the fluid surface

- $h_{link}$: Half-height of the link (used for partial submersion)

- $z_{com}$: Z-coordinate of the link's center of mass

## 3. Integration Schemes
Implemented in `farms_amphibious/control/network.py` via `scipy.integrate.ode`.

- **Scheme:** `dopri5`
- **Method:** Dormand-Prince Runge-Kutta 4(5).
- **Details:** Explicit Runge-Kutta method with adaptive step sizing, chosen for its stability in integrating stiff non-linear equations like coupled CPG oscillators. Bounded in the integration loop to align with the physics time step.

## 4. Coordinate Frame Transforms
Implemented in `farms_mujoco/swimming/drag.pyx`. Uses quaternions for transforming vectors.

**Transforms:**

$$
q_{global \to urdf} = q_{urdf \to global}^*
$$

$$
q_{com \to urdf} = q_{global \to urdf} \cdot q_{com \to global}
$$

$$
q_{urdf \to com} = q_{com \to urdf}^*
$$

**Vector Rotations:**

$$
v_{urdf} = q_{global \to urdf} \cdot v_{global} \cdot q_{global \to urdf}^*
$$

$$
F_{com} = q_{urdf \to com} \cdot F_{urdf} \cdot q_{urdf \to com}^*
$$

Where:

- $q_{global \to urdf}$: Quaternion representing rotation from the global world frame to the URDF link frame

- $q_{urdf \to global}^*$: Complex conjugate (inverse) of the rotation from URDF to global frame

- $q_{com \to global}$: Quaternion representing rotation from the Center of Mass (CoM) frame to the global frame

- $q_{com \to urdf}$: Quaternion representing rotation from the CoM frame to the URDF link frame

- $v_{global}$: Velocity vector in the global coordinate frame

- $v_{urdf}$: Velocity vector rotated into the local URDF coordinate frame

- $F_{urdf}$: Force vector in the local URDF frame

- $F_{com}$: Force vector rotated into the local CoM frame
