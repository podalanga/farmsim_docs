# farms_amphibious.control.ode

!!! note "Source Files"
    - `farms_amphibious/control/ode.pyx` — Cython ODE kernels (compiled to `.pyd`)
    - `farms_amphibious/data/data_cy.pyx` — Cython state containers
    - `farms_amphibious/data/network.py` — Python wrappers and data classes
    - `farms_amphibious/control/network.py` — ODE integrator

FARMS implements a **biologically inspired Central Pattern Generator (CPG)** network modelled as a system of coupled nonlinear oscillators. This is the mathematical engine that generates rhythmic, coordinated locomotion patterns for swimming and walking in amphibious robots. The oscillators are a **Hopf-type** phase-amplitude system — each oscillator has an independent phase and amplitude, coupled through weighted, phase-biased connections.

---

## Mathematical Foundation

### State Vector Layout

The complete network state is stored in a flat 1D array of size `2·N_osc + N_joints` where:

- Indices `[0, N_osc)` → **phases** `φᵢ` (in radians)
- Indices `[N_osc, 2·N_osc)` → **amplitudes** `rᵢ`
- Indices `[2·N_osc, 2·N_osc + N_joints)` → **joint offsets** `δⱼ`

This layout is enforced by `OscillatorNetworkStateCy`:

```python
cpdef DTYPEv1 phases(self, unsigned int iteration):
    return self.array[iteration, :self.n_oscillators]

cpdef DTYPEv1 amplitudes(self, unsigned int iteration):
    return self.array[iteration, self.n_oscillators:2*self.n_oscillators]

cpdef DTYPEv1 offsets(self, unsigned int iteration):
    return self.array[iteration, 2*self.n_oscillators:]
```

### Oscillator Output Function

The neural output of each oscillator — used to drive muscle models — is:

$$
y_i = r_i \cdot (1 + \cos(\varphi_i))
$$

This is computed by `outputs()`:

```python
cpdef np.ndarray outputs(self, unsigned int iteration):
    return self.amplitudes(iteration) * (1 + np.cos(self.phases(iteration)))
```

!!! important "Why This Output Function"
    The output $y_i \in [0, 2r_i]$ is always non-negative, which directly represents the firing rate of a motor neuron. When $\varphi_i = 0$, output is maximum ($2r_i$); when $\varphi_i = \pi$, output is zero. This is the half-wave rectified cosine — a well-established model for motor neuron activity.

---

## Phase Dynamics: `ode_dphase`

The phase ODE is the core of the CPG. It computes `dφᵢ/dt` for each oscillator:

$$
\frac{d\varphi_i}{dt} = \omega_i \cdot \left(1 + A^{mod}_i \cdot \cos\!\left(\varphi_i + \Phi^{mod}_i\right)\right)
+ \sum_{j \in \mathcal{C}(i)} r_j \cdot w_{ji} \cdot \sin\!\left(\varphi_j - \varphi_i - \Delta\varphi_{ji}\right)
$$

Where:

| Symbol | Description | Source |
|--------|-------------|--------|
| $\omega_i$ | Intrinsic angular frequency (rad/s) | `oscillators.c_angular_frequency(iteration, i, drives)` |
| $A^{mod}_i$ | Modular amplitude (frequency modulation depth) | `oscillators.c_modular_amplitudes(i)` |
| $\Phi^{mod}_i$ | Modular phase offset | `oscillators.c_modular_phases(i)` |
| $r_j$ | Amplitude of pre-synaptic oscillator $j$ | `state[n_oscillators + j]` |
| $w_{ji}$ | Coupling weight from $j$ to $i$ | `connectivity.c_weight(i)` |
| $\Delta\varphi_{ji}$ | Desired phase difference (phase bias) | `connectivity.c_desired_phase(i)` |
| $\mathcal{C}(i)$ | Set of connections targeting oscillator $i$ | `osc2osc_map` |

**From `ode.pyx` (lines 45–78):**

```cython
cpdef inline void ode_dphase(...) nogil:
    for i in range(n_oscillators):
        # Intrinsic frequency
        dstate[i] = oscillators.c_angular_frequency(iteration, i, drives)
        if oscillators.c_modular_amplitudes(i) > 1e-3:
            dstate[i] *= (
                1 + oscillators.c_modular_amplitudes(i)*cos(
                    phase(state, i) + oscillators.c_modular_phases(i)
                )
            )
    for i in range(connectivity.n_connections):
        # Neural couplings  (OSC2OSC connections only)
        i0 = connectivity.connections.array[i, 0]   # target
        i1 = connectivity.connections.array[i, 1]   # source
        dstate[i0] += state[n_oscillators+i1]*connectivity.c_weight(i)*sin(
            phase(state, i1) - phase(state, i0) - connectivity.c_desired_phase(i)
        )
```

!!! note "Phase coupling is amplitude-weighted"
    The coupling term uses `state[n_oscillators + i1]` — the **amplitude** of the source oscillator, not 1. This means a silenced oscillator (amplitude → 0) stops exerting influence on its neighbours, which is critical for smooth gait transitions.

---

## Amplitude Dynamics: `ode_damplitude`

Each oscillator's amplitude follows a first-order relaxation toward a drive-dependent target:

$$
\frac{dr_i}{dt} = a_i \cdot \left(R_i^{nom} - r_i\right)
$$

Where:

| Symbol | Description | Source |
|--------|-------------|--------|
| $a_i$ | Convergence rate (rad/s) | `oscillators.c_rate(i)` |
| $R_i^{nom}$ | Nominal amplitude (drive-dependent) | `oscillators.c_nominal_amplitude(iteration, i, drives)` |

**From `ode.pyx` (lines 81–99):**

```cython
cpdef inline void ode_damplitude(...) nogil:
    for i in range(n_oscillators):
        dstate[n_oscillators+i] = oscillators.c_rate(i)*(
            oscillators.c_nominal_amplitude(iteration, i, drives)
            - amplitude(state, i, n_oscillators)
        )
```

The convergence rate `a_i` determines how quickly the oscillator amplitude tracks changes in the descending drive. A high value (e.g., 20 rad/s) gives near-instantaneous tracking; a low value (e.g., 1 rad/s) produces smooth interpolation when switching between gaits.

---

## Drive-Dependent Parameters

Both `ω_i` and `R_i^{nom}` are not fixed — they are **piecewise-linear functions of the descending drive** `d_i`:

$$
\omega_i(d_i) = \text{clamp}\left(g^{\omega}_i \cdot d_i + b^{\omega}_i,\; s^{\omega}_{lo},\; s^{\omega}_{hi}\right), \quad \text{if } d_i \in [lo_i, hi_i]
$$

The drive itself is clamped:

$$
d_i^{eff} = \text{clamp}(d_i, lo_i, hi_i)
$$

The `DriveDependentArrayCy` stores 6 parameters per oscillator: `[gain, bias, low, high, saturation_low, saturation_high]`:

```python
class DriveDependentArray(DriveDependentArrayCy):
    @classmethod
    def from_vectors(cls, gain, bias, low, high, saturation_low, saturation_high):
        return cls(np.array([gain, bias, low, high, saturation_low, saturation_high]))
```

This creates the characteristic "drive-frequency" curve seen in salamander CPG literature: below `low`, the oscillator is silent; above `high`, frequency saturates.

---

## Joint Offset Dynamics: `ode_joints`

The joint offsets `δⱼ` (used by muscle models as equilibrium positions) also evolve as first-order systems:

$$
\frac{d\delta_j}{dt} = a_j^{off} \cdot \left(\delta_j^{des}(d) - \delta_j\right)
$$

**From `ode.pyx` (lines 237–259):**

```cython
cpdef inline void ode_joints(...) nogil:
    for joint_i in range(n_joints):
        dstate[2*n_oscillators+joint_i] = joints.c_rate(joint_i)*(
            joints.c_offset_desired(iteration, joint_i, drives)
            - joint_offset(state, joint_i, n_oscillators)
        )
```

This allows the limb equilibrium angle to ramp smoothly between walking and swimming values as the drive level changes, preventing discontinuous jumps in the reference position.

---

## Sensory Feedback Channels

The FARMS CPG supports **four sensory feedback pathways** that modulate the oscillator dynamics. These add to the `dstate` buffer computed above.

### Stretch Receptor Feedback: `ode_stretch`

Proprioceptive feedback from joint position sensors. Two sub-types:

**Standard stretch:** Direct additive frequency/amplitude modulation:

$$
\frac{d\varphi_{i0}}{dt} \mathrel{+}= w \cdot \theta_{i1} \quad (\text{STRETCH2FREQ})
$$

$$
\frac{dr_{i0}}{dt} \mathrel{+}= w \cdot \theta_{i1} \quad (\text{STRETCH2AMP})
$$

**Tegotae stretch:** Multiplied by $\sin(\varphi_i)$ — this is phase-dependent feedback:

$$
\frac{d\varphi_{i0}}{dt} \mathrel{+}= w \cdot \theta_{i1} \cdot \sin(\varphi_{i0}) \quad (\text{STRETCH2FREQTEGOTAE})
$$

!!! note "Tegotae feedback origin"
    Tegotae (手応え, Japanese for "tactile response") is a control strategy introduced by Owaki & Ishiguro (2017) where each limb CPG is locally modulated by the ground reaction force, producing emergent gait patterns without central coordination. FARMS implements this via `sin(phase)` multiplication.

**From `ode.pyx` (lines 102–156):**

```cython
if connection_type == ConnectionType.STRETCH2FREQTEGOTAE:
    dstate[i0] += (
        joints2osc_map.c_weight(i)
        * joints.position_cy(iteration, i1)
        * sin(state[i0])   # Phase-dependent — Tegotae
    )
elif connection_type == ConnectionType.STRETCH2FREQ:
    dstate[i0] += (
        joints2osc_map.c_weight(i)
        * joints.position_cy(iteration, i1)
    )
```

### Contact Reaction Feedback: `ode_contacts`

Ground reaction forces modulate oscillator frequency:

$$
\|F_{contact}\| = \sqrt{F_x^2 + F_y^2 + F_z^2}
$$

$$
\frac{d\varphi_{i0}}{dt} \mathrel{+}= w \cdot \|F_{contact,\,i1}\| \quad (\text{REACTION2FREQ})
$$

With Tegotae variant:

$$
\frac{d\varphi_{i0}}{dt} \mathrel{+}= w \cdot \|F_{contact,\,i1}\| \cdot \sin(\varphi_{i0}) \quad (\text{REACTION2FREQTEGOTAE})
$$

**From `ode.pyx` (lines 159–193):**

```cython
contact_reaction = sqrt(
    contacts.c_total_x(iteration, i1)**2
    + contacts.c_total_y(iteration, i1)**2
    + contacts.c_total_z(iteration, i1)**2
)
if connection_type == ConnectionType.REACTION2FREQ:
    dstate[i0] += contacts2osc_map.c_weight(i) * contact_reaction
elif connection_type == ConnectionType.REACTION2FREQTEGOTAE:
    dstate[i0] += (
        contacts2osc_map.c_weight(i)
        * contact_reaction
        * sin(state[i0])
    )
```

### External Force (Xfrc) Feedback: `ode_xfrc`

Lateral hydrodynamic forces (from `xfrc_applied` in MuJoCo) modulate oscillators:

$$
\frac{d\varphi_{i0}}{dt} \mathrel{+}= w \cdot |F_{y,\,i1}| \quad (\text{LATERAL2FREQ})
$$

$$
\frac{dr_{i0}}{dt} \mathrel{+}= w \cdot |F_{y,\,i1}| \quad (\text{LATERAL2AMP})
$$

Only the **Y-component** (lateral force) is used, since this is the dominant drag direction during undulatory swimming.

---

## The Complete ODE: `ode_oscillators_sparse`

The master function assembles all contributions in order:

```cython
cpdef inline DTYPEv1 ode_oscillators_sparse(
    DTYPE time, DTYPEv1 state, DTYPEv1 dstate,
    unsigned int iteration, AmphibiousDataCy data,
    unsigned int nosfb=0,  # No sensory feedback flag
) nogil:
    # 1. Phase dynamics (intrinsic + coupling)
    ode_dphase(iteration, state, dstate, data.network.drives,
               data.network.oscillators, data.network.osc2osc_map)
    # 2. Amplitude dynamics (drive-dependent relaxation)
    ode_damplitude(iteration, state, dstate, data.network.drives,
                   data.network.oscillators)
    # 3. Joint offset dynamics
    ode_joints(iteration, state, dstate, data.network.drives,
               data.joints, data.network.oscillators.n_oscillators)
    # 4. Sensory feedback (optional)
    if nosfb:
        return dstate
    ode_stretch(...)
    ode_contacts(...)
    ode_xfrc(...)
    return dstate
```

Setting `nosfb=1` bypasses all sensory feedback, giving a pure open-loop CPG — useful for in-water locomotion experiments where only the descending drive matters.

---

## Oscillator Naming Convention

The `AmphibiousConvention` class defines the canonical name for each oscillator, derived from its role in the body plan. This is critical for constructing connectivity matrices.

For a robot with `n_joints_body` body joints and `n_legs` legs (with `n_dof_legs` DOF each):

### Body Oscillators

With two oscillators per joint (flexor/extensor):

```
osc_body_0_L   → index 0   (left/flexor, joint 0)
osc_body_0_R   → index 1   (right/extensor, joint 0)
osc_body_1_L   → index 2
osc_body_1_R   → index 3
...
```

With single oscillator per joint (`single_osc_body=True`):

```
osc_body_0     → index 0
osc_body_1     → index 1
```

### Leg Oscillators

Pattern: `osc_leg_{leg_i}_{side}_{joint_i}_{osc_side}`:

```
osc_leg_0_L_0_0  → first leg, left side, first DOF, flexor
osc_leg_0_L_0_1  → first leg, left side, first DOF, extensor
```

### Index Computation

From `convention.py`:

```python
def legosc2index(self, leg_i, side_i, joint_i, side=0):
    return self.leg_osc_indices(leg_i=leg_i, side_i=side_i, joint_i=joint_i)[side]

def leg_osc_indices(self, **kwargs):
    index = (
        self.n_osc_body()
        + leg_i * 2 * n_legs_dof * leg_opj  # 2 sides per leg pair
        + side_i * n_legs_dof * leg_opj
        + joint_i * leg_opj
    )
    return list(range(index, index + leg_opj))
```

---

## Oscillator Connectivity Matrix

The `OscillatorConnectivity` class stores the coupling structure as a sparse `[N_connections, 3]` integer array:

| Column | Content |
|--------|---------|
| `[i, 0]` | Target oscillator index `i0` |
| `[i, 1]` | Source oscillator index `i1` |
| `[i, 2]` | Connection type (`OSC2OSC`) |

With parallel arrays:
- `weights[i]` → coupling strength $w_{ji}$
- `desired_phases[i]` → phase bias $\Delta\varphi_{ji}$

**Construction from a list of dicts:**

```python
connectivity = [
    {'in': 'osc_body_0_L', 'out': 'osc_body_1_L', 'type': 'OSC2OSC',
     'weight': 10.0, 'phase_bias': 0.628},
    ...
]
OscillatorConnectivity.from_connectivity(connectivity, map1=osc_name2index, map2=osc_name2index)
```

---

## `NetworkODE`: Integration of the CPG

The `NetworkODE` class wraps the `ode_oscillators_sparse` kernel with a SciPy ODE solver:

```python
class NetworkODE(AnimatNetwork):
    def __init__(self, data, integrator='dopri5', nsteps=1000, max_step, verbosity):
        self.solver = scipy.integrate.ode(ode_wrapper)
        self.solver.set_integrator('dopri5', nsteps=nsteps, max_step=max_step)
```

### Why `dopri5`?

`dopri5` is the **Dormand-Prince 4th/5th-order Runge-Kutta** with adaptive step size. It is the same algorithm as MATLAB's `ode45`. It is chosen because:

1. **Explicit method** — works well for the non-stiff phase dynamics
2. **Adaptive stepping** — automatically handles the fast transients when drive changes abruptly
3. **Embedded error estimate** — 4th order result for stepping, 5th order for error estimation, giving efficient accuracy control
4. **Low overhead** — uses only 6 function evaluations per step (Butcher tableau with FSAL property)

### Integration Step Logic

```python
def step(self, iteration, time, timestep, checks=False, strict=False):
    if iteration == 0:
        self.copy_next_drive(iteration)
        return
    if iteration % self.modulo:
        # Sub-step: copy previous state
        self.data.state.array[iteration, :] = self.data.state.array[iteration-1, :]
        return
    # Full integration step
    self.solver.set_f_params(dstate, iteration, self.data)
    while self.solver.successful() and self.solver.t < time + 0.99*timestep:
        self.data.state.array[iteration, :] = self.solver.integrate(time + timestep)
    if not self.solver.successful():
        if strict:
            raise IntegrationException(...)
        else:
            self.solver.set_initial_value(...)  # Reset solver
    if iteration < self.n_iterations - 1:
        self.copy_next_drive(iteration)
```

The `0.99*timestep` tolerance prevents the solver from stepping past the physics timestep boundary, which would invalidate the sensory feedback values read at `iteration`.

---

## Complete Connection Type Reference

All 19 connection types defined in `data_cy.pyx` and named in `CONNECTIONTYPENAMES`:

| Enum Value | Name | Effect | Where Applied |
|---|---|---|---|
| 0 | `OSC2OSC` | Phase coupling between oscillators | `ode_dphase` |
| 1 | `DRIVE2OSC` | Direct drive-to-oscillator mapping | Data structure |
| 2 | `POS2FREQ` | Joint position → frequency | `ode_stretch` |
| 3 | `VEL2FREQ` | Joint velocity → frequency | `ode_stretch` |
| 4 | `TOR2FREQ` | Joint torque → frequency | `ode_stretch` |
| 5 | `POS2AMP` | Joint position → amplitude | `ode_stretch` |
| 6 | `VEL2AMP` | Joint velocity → amplitude | `ode_stretch` |
| 7 | `TOR2AMP` | Joint torque → amplitude | `ode_stretch` |
| 8 | `STRETCH2FREQTEGOTAE` | Position × sin(φ) → frequency (Tegotae) | `ode_stretch` |
| 9 | `STRETCH2AMPTEGOTAE` | Position × sin(φ) → amplitude (Tegotae) | `ode_stretch` |
| 10 | `STRETCH2FREQ` | Position → frequency (linear) | `ode_stretch` |
| 11 | `STRETCH2AMP` | Position → amplitude (linear) | `ode_stretch` |
| 12 | `REACTION2FREQ` | Contact force magnitude → frequency | `ode_contacts` |
| 13 | `REACTION2AMP` | Contact force magnitude → amplitude | `ode_contacts` |
| 14 | `REACTION2FREQTEGOTAE` | Contact × sin(φ) → frequency (Tegotae) | `ode_contacts` |
| 15 | `FRICTION2FREQ` | Friction force → frequency | (not yet in ode.pyx) |
| 16 | `FRICTION2AMP` | Friction force → amplitude | (not yet in ode.pyx) |
| 17 | `LATERAL2FREQ` | Lateral xfrc magnitude → frequency | `ode_xfrc` |
| 18 | `LATERAL2AMP` | Lateral xfrc magnitude → amplitude | `ode_xfrc` |

---

## Class Reference

### `OscillatorNetworkStateCy`

**Source:** `farms_amphibious/data/data_cy.pyx`  
**Base:** `DoubleArray2D`

Container for the time-history of the full CPG state.

| Attribute | Type | Description |
|---|---|---|
| `array` | `float64[n_iter, state_size]` | Full state history array |
| `n_oscillators` | `int` | Number of oscillators (partitions the array) |

| Method | Returns | Description |
|---|---|---|
| `phases(iteration)` | `float64[N_osc]` | Phase values at given iteration |
| `amplitudes(iteration)` | `float64[N_osc]` | Amplitude values |
| `offsets(iteration)` | `float64[N_joints]` | Joint offset values |
| `outputs(iteration)` | `float64[N_osc]` | Neural outputs: $r_i(1+\cos\varphi_i)$ |
| `phases_all()` | `float64[n_iter, N_osc]` | All phase history |
| `amplitudes_all()` | `float64[n_iter, N_osc]` | All amplitude history |
| `outputs_all()` | `float64[n_iter, N_osc]` | All neural output history |

### `OscillatorNetworkState`

**Source:** `farms_amphibious/data/network.py`  
**Base:** `OscillatorNetworkStateCy`

Python-level extension. Key factory:

```python
@classmethod
def from_initial_state(cls, initial_state, n_iterations, n_oscillators):
    state_array = np.full([n_iterations, len(initial_state)], 0, dtype=float64)
    state_array[0, :] = initial_state
    return cls(array=state_array, n_oscillators=n_oscillators)
```

The initial state comes from `AmphibiousOptions.state_init()`:

```python
def state_init(self):
    return (
        [osc.initial_phase for osc in network.oscillators]     # N_osc phases
        + [osc.initial_amplitude for osc in network.oscillators]  # N_osc amplitudes
        + [joint.initial[0] for joint in active_joints]          # N_joints offsets
    )
```

### `Oscillators`

**Source:** `farms_amphibious/data/network.py`  
**Base:** `OscillatorsCy`

Stores the per-oscillator parameters array.

| Field | Type | Description |
|---|---|---|
| `names` | `list[str]` | Oscillator names (from convention) |
| `drive2osc_map` | `IntegerArray1D[N_osc]` | Which drive index modulates each oscillator |
| `intrinsic_frequencies` | `DriveDependentArray[N_osc, 6]` | Frequency vs drive curve |
| `nominal_amplitudes` | `DriveDependentArray[N_osc, 6]` | Amplitude vs drive curve |
| `rates` | `DoubleArray1D[N_osc]` | Amplitude convergence rates |
| `modular_phases` | `DoubleArray1D[N_osc]` | Phase offset for modular frequency |
| `modular_amplitudes` | `DoubleArray1D[N_osc]` | Depth of frequency modulation |

### `DriveDependentArray`

**Source:** `farms_amphibious/data/network.py`  
**Base:** `DriveDependentArrayCy`

Encodes piecewise-linear functions of the drive signal. The 6 parameters:

| Index | Parameter | Description |
|---|---|---|
| 0 | `gain` | Slope of the linear mapping |
| 1 | `bias` | Intercept of the linear mapping |
| 2 | `low` | Drive below this → use saturation_low |
| 3 | `high` | Drive above this → use saturation_high |
| 4 | `saturation_low` | Value when drive < low |
| 5 | `saturation_high` | Value when drive > high |

The effective formula is:

$$
f(d) = \begin{cases}
s_{lo} & \text{if } d < lo \\
\text{clamp}(g \cdot d + b,\; s_{lo},\; s_{hi}) & \text{if } lo \leq d \leq hi \\
s_{hi} & \text{if } d > hi
\end{cases}
$$

### `OscillatorConnectivity`

**Source:** `farms_amphibious/data/network.py`  
**Base:** `OscillatorsConnectivityCy`

| Field | Shape | Description |
|---|---|---|
| `connections` | `[N_conn, 3]` | `[target, source, type]` integer array |
| `weights` | `[N_conn]` | Coupling weights $w_{ji}$ |
| `desired_phases` | `[N_conn]` | Phase biases $\Delta\varphi_{ji}$ (radians) |

---

## Related Pages

- [Ekeberg Muscle Model](ekeberg_muscle.md) — How CPG outputs drive joint torques
- [Position Muscle & Phase Controllers](joint_controllers.md) — How phases map to position commands
- [Descending Drive](farms_amphibious_drive.md) — How drive signals are computed
- [NetworkODE](farms_amphibious_network.md) — Integration loop details
