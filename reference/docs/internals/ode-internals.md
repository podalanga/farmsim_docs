# ODE Internals: CPG Network Integration

This page documents the Cython ODE functions (`farms_amphibious/control/ode.pyx`, 320 lines) and the Python integration wrapper (`farms_amphibious/control/network.py`, 121 lines) that together form the mathematical core of the amphibious locomotion controller. These functions compute the derivatives of the CPG oscillator network state and integrate them forward in time.

## Source files covered

| File | Lines | Purpose |
|---|---|---|
| `farms_amphibious/control/ode.pyx` | 320 | Cython ODE functions: phase, amplitude, joint, sensory feedback |
| `farms_amphibious/control/network.py` | 121 | `AnimatNetwork` (ABC), `NetworkODE` — scipy.integrate.ode wrapper |
| `farms_amphibious/data/data_cy.pyx` | — | `ConnectionType` enum, `AmphibiousDataCy` cdef class |

## Call graph / entry points

```
Simulation.run()
  └─ ExperimentTask.before_step()
       └─ AmphibiousController.before_step()
            └─ network.step(iteration, time, timestep)
                 └─ solver.integrate(time + timestep)
                      └─ ode_oscillators_sparse(time, state, dstate, iteration, data)
                           ├─ ode_dphase()       — phase derivatives
                           ├─ ode_damplitude()   — amplitude derivatives
                           ├─ ode_joints()       — joint offset derivatives
                           ├─ ode_stretch()      — proprioceptive feedback (if nosfb==0)
                           ├─ ode_contacts()     — tactile feedback (if nosfb==0)
                           └─ ode_xfrc()         — external force feedback (if nosfb==0)
```

## State vector layout

The state vector is a 1D array of doubles (Cython type `DTYPEv1`, which is a typed memoryview `double[:]`). The layout, determined by `AmphibiousConvention.n_states()`, is:

```
[ phases         ]  indices 0 .. n_oscillators-1
[ amplitudes     ]  indices n_oscillators .. 2*n_oscillators-1
[ joints_offsets ]  indices 2*n_oscillators .. 2*n_oscillators+n_joints_active-1
```

Total length: `2 * n_oscillators + n_joints_active` (see [AmphibiousConvention](amphibious-convention.md) for how these are computed).

The derivative vector (`dstate`) has the same layout. It is zeroed before each call to `ode_oscillators_sparse` — actually, it is NOT zeroed by the ODE function itself. The `dstate` array is allocated once in `NetworkODE.__init__` and reused. The scipy integrator calls `ode_oscillators_sparse` which writes into `dstate`. The `dstate` is passed via `set_f_params`.

### Inline accessors

Three `cdef inline` helper functions provide typed access to state components:

```cython
cdef inline DTYPE phase(DTYPEv1 state, unsigned int index) nogil:
    """Phase: state[index]"""
    return state[index]

cdef inline DTYPE amplitude(DTYPEv1 state, unsigned int index, unsigned int n_oscillators) nogil:
    """Amplitude: state[index + n_oscillators]"""
    return state[index + n_oscillators]

cdef inline DTYPE joint_offset(DTYPEv1 state, unsigned int index, unsigned int n_oscillators) nogil:
    """Joint offset: state[index + 2*n_oscillators]"""
    return state[index + 2 * n_oscillators]
```

These are inlined at the C level, so there is no function call overhead.

## The main entry point: `ode_oscillators_sparse`

```cython
cpdef inline DTYPEv1 ode_oscillators_sparse(
    DTYPE time,              # Current simulation time [s]
    DTYPEv1 state,           # State vector (phases, amplitudes, offsets)
    DTYPEv1 dstate,          # Derivative vector (output, same layout as state)
    unsigned int iteration,  # Current iteration number
    AmphibiousDataCy data,   # Cython data container with all network/sensor data
    unsigned int nosfb=0,    # No sensory feedback flag (0 = feedback enabled)
) nogil
```

This function is the right-hand side (RHS) of the ODE system. It is called by `scipy.integrate.ode` (via `NetworkODE.step`) to integrate the CPG state forward. The `nogil` decorator means it runs without holding the Python GIL, enabling pure C-level execution.

### Execution order

```cython
cpdef inline DTYPEv1 ode_oscillators_sparse(DTYPE time, DTYPEv1 state, DTYPEv1 dstate,
    unsigned int iteration, AmphibiousDataCy data, unsigned int nosfb=0) nogil:
    """Complete CPG network ODE"""
    ode_dphase(iteration, state, dstate, data.network.drives, data.network.oscillators,
              data.network.osc2osc_map)
    ode_damplitude(iteration, state, dstate, data.network.drives, data.network.oscillators)
    ode_joints(iteration, state, dstate, data.network.drives, data.joints,
               data.network.oscillators.n_oscillators)
    if nosfb:
        return dstate
    ode_stretch(iteration, state, dstate, data.sensors.joints, data.network.joints2osc_map,
                data.network.oscillators.n_oscillators)
    ode_contacts(iteration, state, dstate, data.sensors.contacts, data.network.contacts2osc_map)
    ode_xfrc(iteration, state, dstate, data.sensors.xfrc, data.network.xfrc2osc_map,
             data.network.oscillators.n_oscillators)
    return dstate
```

**Key behavior**: The function calls sub-functions in a specific order, each ADDING to `dstate` (using `+=`). The `dstate` is not zeroed within this function. The three core dynamics (dphase, damplitude, djoints) are always computed. The three sensory feedback functions are only computed when `nosfb == 0`.

**The `nosfb` flag**: When set to non-zero, sensory feedback is completely skipped. This is used during initialization or debugging to isolate the intrinsic CPG dynamics from sensory-driven modulation.

## Phase derivative: `ode_dphase`

```cython
cpdef inline void ode_dphase(
    unsigned int iteration,
    DTYPEv1 state,
    DTYPEv1 dstate,
    DriveArrayCy drives,
    OscillatorsCy oscillators,
    OscillatorsConnectivityCy connectivity,
) nogil
```

### Mathematical formula

\[ \dot{\theta}_i = \omega_i \cdot (1 + \text{mod\_amp}_i \cdot \cos(\theta_i + \text{mod\_phase}_i)) + \sum_j R_j \cdot w_{ij} \cdot \sin(\theta_j - \theta_i - \phi_{ij}) \]

### Line-by-line code walkthrough

```cython
cdef unsigned int i, i0, i1, n_oscillators = oscillators.n_oscillators

# Part 1: Intrinsic frequency and modulation
for i in range(n_oscillators):
    # Intrinsic frequency (drive-dependent)
    dstate[i] = oscillators.c_angular_frequency(iteration, i, drives)
    # Modular amplitude coupling (skip if near zero)
    if oscillators.c_modular_amplitudes(i) > 1e-3:
        dstate[i] *= (
            1 + oscillators.c_modular_amplitudes(i) * cos(
                phase(state, i) + oscillators.c_modular_phases(i)
            )
        )

# Part 2: Neural couplings
for i in range(connectivity.n_connections):
    i0 = connectivity.connections.array[i, 0]  # Source oscillator (receives coupling)
    i1 = connectivity.connections.array[i, 1]  # Target oscillator (provides signal)
    dstate[i0] += state[n_oscillators + i1] * connectivity.c_weight(i) * sin(
        phase(state, i1) - phase(state, i0)
        - connectivity.c_desired_phase(i)
    )
```

**Part 1 walkthrough**:

- `oscillators.c_angular_frequency(iteration, i, drives)`: This is a `cdef` method on `OscillatorsCy` that computes the drive-dependent angular frequency for oscillator `i`. It reads the drive value from `drives` at the given `iteration` and applies a piecewise-linear function (see [Data Allocation Internals](data-allocation-internals.md) for the 6-parameter drive-dependent array) to map the drive value to a frequency.

- The modular amplitude check `if oscillators.c_modular_amplitudes(i) > 1e-3`: This is a threshold to skip the modulation term when it's effectively zero. The threshold of `1e-3` is hardcoded. If the modular amplitude is below this, the frequency is just the intrinsic frequency. If above, the frequency is multiplied by `(1 + mod_amp * cos(phase + mod_phase))`, which creates a phase-dependent frequency modulation.

**Part 2 walkthrough**:

- The coupling loop iterates over ALL connections in the sparse connectivity matrix. Each connection is a row in `connectivity.connections.array` with at least 2 columns: `[source_osc, target_osc, ...]`.

- `i0 = connectivity.connections.array[i, 0]`: The oscillator that RECEIVES the coupling (its derivative is modified).

- `i1 = connectivity.connections.array[i, 1]`: The oscillator that PROVIDES the coupling signal.

- `state[n_oscillators + i1]`: The **amplitude** of the target oscillator `i1`. The coupling is amplitude-weighted — oscillators with larger amplitudes have stronger influence.

- `connectivity.c_weight(i)`: The connection weight for connection `i`.

- `connectivity.c_desired_phase(i)`: The desired phase bias \(\phi_{ij}\) for this connection.

- `sin(phase(state, i1) - phase(state, i0) - connectivity.c_desired_phase(i))`: The phase difference term. This is the standard Kuramoto coupling: the coupling drives the phase difference toward the desired phase bias.

**Important note on indexing**: `i0` is the oscillator whose derivative is modified (the "source" in terms of influence flow), but in the connectivity array it's the first column. The second column `i1` provides the signal. This means the connectivity array is structured as `[influenced_osc, influencing_osc, ...]`.

## Amplitude derivative: `ode_damplitude`

```cython
cpdef inline void ode_damplitude(
    unsigned int iteration,
    DTYPEv1 state,
    DTYPEv1 dstate,
    DriveArrayCy drives,
    OscillatorsCy oscillators,
) nogil
```

### Mathematical formula

\[ \dot{R}_i = \text{rate}_i \cdot (R^{\text{nominal}}_i(\text{drive}) - R_i) \]

### Code walkthrough

```cython
cdef unsigned int i, n_oscillators = oscillators.n_oscillators
for i in range(n_oscillators):
    # rate * (nominal_amplitude - current_amplitude)
    dstate[n_oscillators + i] = oscillators.c_rate(i) * (
        oscillators.c_nominal_amplitude(iteration, i, drives)
        - amplitude(state, i, n_oscillators)
    )
```

This is a first-order linear approach to the nominal amplitude. The `rate` parameter controls how fast the amplitude converges. The nominal amplitude is drive-dependent — it is computed via `c_nominal_amplitude(iteration, i, drives)` which applies the same piecewise-linear drive-dependent function as the frequency.

The amplitude is stored at `state[n_oscillators + i]` and its derivative at `dstate[n_oscillators + i]`.

## Joint offset derivative: `ode_joints`

```cython
cpdef inline void ode_joints(
    unsigned int iteration,
    DTYPEv1 state,
    DTYPEv1 dstate,
    DriveArrayCy drives,
    JointsControlArrayCy joints,
    unsigned int n_oscillators,
) nogil
```

### Mathematical formula

\[ \dot{\phi}_j = \text{rate}_j \cdot (\phi^{\text{desired}}_j(\text{drive}) - \phi_j) \]

### Code walkthrough

```cython
cdef unsigned int joint_i, n_joints = joints.c_n_joints()
for joint_i in range(n_joints):
    # rate * (desired_offset - current_offset)
    dstate[2 * n_oscillators + joint_i] = joints.c_rate(joint_i) * (
        joints.c_offset_desired(iteration, joint_i, drives)
        - joint_offset(state, joint_i, n_oscillators)
    )
```

Same first-order convergence as the amplitude equation, but for joint offsets. The desired offset is drive-dependent. The joint offset is stored at `state[2 * n_oscillators + joint_i]`.

**Note**: `n_joints` comes from `joints.c_n_joints()`, which returns the number of **active** joints (body + legs, no passive). This matches the state vector layout where only active joints have offset entries.

## Sensory feedback: `ode_stretch`

```cython
cpdef inline void ode_stretch(
    unsigned int iteration,
    DTYPEv1 state,
    DTYPEv1 dstate,
    JointSensorArrayCy joints,
    JointsConnectivityCy joints2osc_map,
    unsigned int n_oscillators,
) nogil
```

Proprioceptive feedback from joint positions into the oscillator network. This implements sensory feedback where the physical joint position (measured from the physics simulation) influences the CPG dynamics.

### Code walkthrough

```cython
cdef unsigned int i, i0, i1, connection_type
for i in range(joints2osc_map.n_connections):
    i0 = joints2osc_map.connections.array[i, 0]  # Oscillator index
    i1 = joints2osc_map.connections.array[i, 1]  # Joint index
    connection_type = joints2osc_map.connections.array[i, 2]

    if connection_type == ConnectionType.STRETCH2FREQTEGOTAE:
        # stretch_weight * joint_position * sin(phase)
        dstate[i0] += (
            joints2osc_map.c_weight(i)
            * joints.position_cy(iteration, i1)
            * sin(state[i0])  # For Tegotae
        )
    elif connection_type == ConnectionType.STRETCH2AMPTEGOTAE:
        # stretch_weight * joint_position * sin(phase)
        dstate[n_oscillators + i0] += (
            joints2osc_map.c_weight(i)
            * joints.position_cy(iteration, i1)
            * sin(state[i0])  # For Tegotae
        )
    elif connection_type == ConnectionType.STRETCH2FREQ:
        # stretch_weight * joint_position
        dstate[i0] += (
            joints2osc_map.c_weight(i)
            * joints.position_cy(iteration, i1)
        )
    elif connection_type == ConnectionType.STRETCH2AMP:
        # stretch_weight * joint_position
        dstate[n_oscillators + i0] += (
            joints2osc_map.c_weight(i)
            * joints.position_cy(iteration, i1)
        )
    else:
        printf('Joint connection %i of type %i is incorrect, should be %i, %i, %i or %i\n',
            i, connection_type,
            ConnectionType.STRETCH2FREQ, ConnectionType.STRETCH2AMP,
            ConnectionType.STRETCH2FREQTEGOTAE, ConnectionType.STRETCH2AMPTEGOTAE)
```

### Connection types

Each connection in `joints2osc_map.connections.array` is a 3-column row: `[oscillator_index, joint_index, connection_type]`.

| Connection type | Target | Formula | Biological meaning |
|---|---|---|---|
| `STRETCH2FREQ` | Phase derivative | `dstate[i0] += w * joint_pos` | Joint stretch directly modulates frequency |
| `STRETCH2AMP` | Amplitude derivative | `dstate[n_osc+i0] += w * joint_pos` | Joint stretch modulates amplitude |
| `STRETCH2FREQTEGOTAE` | Phase derivative | `dstate[i0] += w * joint_pos * sin(phase)` | Phase-dependent stretch → frequency (Tegotae) |
| `STRETCH2AMPTEGOTAE` | Amplitude derivative | `dstate[n_osc+i0] += w * joint_pos * sin(phase)` | Phase-dependent stretch → amplitude (Tegotae) |

**Tegotae feedback**: The Tegotae variants multiply by `sin(state[i0])` (the current phase of the receiving oscillator). This implements a biologically inspired feedback rule where the effect of sensory input depends on the current phase of the oscillator. The name "Tegotae" comes from the Japanese for "disagreeability" — the feedback is strongest when it opposes the current oscillator state, creating self-stabilizing dynamics.

**Error handling**: If the connection type is not one of the four valid types, a `printf` message is printed to stderr, but execution continues without modifying `dstate`. This is a silent error in `nogil` mode — it cannot raise a Python exception.

**`joints.position_cy(iteration, i1)`**: This is a `cdef` method on `JointSensorArrayCy` that returns the joint position at the given iteration. It reads from the pre-filled sensor data array (filled by `ExperimentTask.update_sensors()` from the physics simulation).

## Sensory feedback: `ode_contacts`

```cython
cpdef inline void ode_contacts(
    unsigned int iteration,
    DTYPEv1 state,
    DTYPEv1 dstate,
    ContactsArrayCy contacts,
    ContactsConnectivityCy contacts2osc_map,
) nogil
```

Tactile feedback from ground contact forces into the oscillator network.

### Code walkthrough

```cython
cdef DTYPE contact_reaction
cdef unsigned int i, i0, i1, connection_type
for i in range(contacts2osc_map.n_connections):
    i0 = contacts2osc_map.connections.array[i, 0]  # Oscillator index
    i1 = contacts2osc_map.connections.array[i, 1]  # Contact sensor index
    connection_type = contacts2osc_map.connections.array[i, 2]

    # Contact reaction magnitude = Euclidean norm of total contact force
    contact_reaction = sqrt(
        contacts.c_total_x(iteration, i1) * contacts.c_total_x(iteration, i1)
        + contacts.c_total_y(iteration, i1) * contacts.c_total_y(iteration, i1)
        + contacts.c_total_z(iteration, i1) * contacts.c_total_z(iteration, i1)
    )

    if connection_type == ConnectionType.REACTION2FREQ:
        dstate[i0] += contacts2osc_map.c_weight(i) * contact_reaction
    elif connection_type == ConnectionType.REACTION2FREQTEGOTAE:
        dstate[i0] += contacts2osc_map.c_weight(i) * contact_reaction * sin(state[i0])
```

### Contact reaction computation

The contact reaction is the **magnitude** (L2 norm) of the total contact force vector at the contact sensor. This is computed inline using `sqrt` from `libc.math` (imported at the top of the file: `from libc.math cimport M_PI, sin, cos, fabs, fmax, fmod, sqrt`).

Note that the code manually computes `x*x + y*y + z*z` instead of using a dot product function — this is for performance in `nogil` mode.

### Connection types

| Connection type | Target | Formula |
|---|---|---|
| `REACTION2FREQ` | Phase derivative | `dstate[i0] += w * contact_reaction` |
| `REACTION2FREQTEGOTAE` | Phase derivative | `dstate[i0] += w * contact_reaction * sin(phase)` |

**Note**: There is NO `REACTION2AMP` or `REACTION2AMPTEGOTAE` — contact feedback only affects phase/frequency, not amplitude. This is a design choice: contacts primarily modulate gait timing, not amplitude.

**No error handling**: Unlike `ode_stretch` and `ode_xfrc`, `ode_contacts` does NOT have an `else` branch for invalid connection types. If an invalid type is encountered, it is silently ignored.

## Sensory feedback: `ode_xfrc`

```cython
cpdef inline void ode_xfrc(
    unsigned int iteration,
    DTYPEv1 state,
    DTYPEv1 dstate,
    XfrcArrayCy xfrc,
    XfrcConnectivityCy xfrc2osc_map,
    unsigned int n_oscillators,
) nogil
```

External force feedback (e.g., lateral water currents) into the oscillator network.

### Code walkthrough

```cython
cdef DTYPE xfrc_force
cdef unsigned int i, i0, i1, connection_type
for i in range(xfrc2osc_map.n_connections):
    i0 = xfrc2osc_map.connections.array[i, 0]  # Oscillator index
    i1 = xfrc2osc_map.connections.array[i, 1]  # Xfrc sensor index
    connection_type = xfrc2osc_map.connections.array[i, 2]

    # Only the y-component (lateral) of external force is used
    xfrc_force = fabs(xfrc.c_force_y(iteration, i1))

    if connection_type == ConnectionType.LATERAL2FREQ:
        dstate[i0] += xfrc2osc_map.c_weights(i) * xfrc_force
    elif connection_type == ConnectionType.LATERAL2AMP:
        dstate[n_oscillators + i0] += xfrc2osc_map.c_weights(i) * xfrc_force
    else:
        printf('Xfrc connection %i of type %i is incorrect, should be %i or %i instead\n',
            i, connection_type,
            ConnectionType.LATERAL2FREQ, ConnectionType.LATERAL2AMP)
```

### Key observations

1. **Only the y-component** of the external force is used (`xfrc.c_force_y()`). This is because `ode_xfrc` is designed for **lateral** perturbations — forces perpendicular to the direction of motion. The x and z components are ignored.

2. **`fabs` (absolute value)** is used, not the raw signed value. This means the feedback is always positive regardless of the direction of the lateral force.

3. **`c_weights` (plural)** is used here, unlike `c_weight` (singular) in the other feedback functions. This appears to be a naming inconsistency in the source code, but both access the same underlying weight array.

### Connection types

| Connection type | Target | Formula |
|---|---|---|
| `LATERAL2FREQ` | Phase derivative | `dstate[i0] += w * fabs(force_y)` |
| `LATERAL2AMP` | Amplitude derivative | `dstate[n_osc+i0] += w * fabs(force_y)` |

## Integration: `NetworkODE`

### Class hierarchy

```
ABC (abc.ABC)
  └─ AnimatNetwork (abstract)
       └─ NetworkODE
```

### `AnimatNetwork` (abstract base class)

```python
class AnimatNetwork(ABC):
    def __init__(self, data, n_iterations):
        super().__init__()
        self.data: AnimatData = data
        self.n_iterations = n_iterations

    @abstractmethod
    def step(self, iteration: int, time: float, timestep: float, **kwargs):
        """Step function called at each simulation iteration"""
```

### `NetworkODE` constructor

```python
class NetworkODE(AnimatNetwork):
    def __init__(self, data, integrator='dopri5', **kwargs):
        state_array = data.state.array
        self.modulo: int = kwargs.pop('modulo', 1)
        super().__init__(data=data, n_iterations=np.shape(state_array)[0])
        self.dstate = np.zeros_like(data.state.array[0, :])
        self.ode: Callable = kwargs.pop('ode', ode_oscillators_sparse)
        self.integrator = integrator
        self.integrator_kwargs = kwargs
        self.solver: ODE = integrate.ode(f=self.ode)
        self.initialize_episode()
```

### Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `data` | `AmphibiousData` | required | Data container with pre-allocated state arrays |
| `integrator` | str | `'dopri5'` | scipy integrator name (Dormand-Prince 5(4)) |
| `modulo` | int | 1 | Controller runs every `modulo` iterations (1 = every step) |
| `ode` | Callable | `ode_oscillators_sparse` | The ODE right-hand side function |

**Remaining kwargs** are passed to `self.solver.set_integrator()`. In practice, `AmphibiousController.from_options` constructs `NetworkODE` with `nsteps=1000` and `max_step=timestep`.

**`self.dstate`**: A zero-initialized array with the same shape as one row of the state array. This is the derivative vector that gets passed to the ODE function via `set_f_params`.

### `initialize_episode()`

```python
def initialize_episode(self):
    self.solver: ODE = integrate.ode(f=self.ode)
    self.solver.set_integrator(self.integrator, **self.integrator_kwargs)
    self.solver.set_initial_value(y=self.data.state.array[0, :], t=0.0)
    self.data.state.array[1:, :] = 0
    self.dstate[:] = 0
```

- Creates a **new** scipy ODE solver instance (the old one is discarded).
- Sets the integrator with its kwargs.
- Sets the initial state to `data.state.array[0, :]` (the first row of the pre-allocated state array, which was loaded from YAML initial conditions).
- **Zeroes all future state rows** (`array[1:, :] = 0`). This is important — if you stored custom initial conditions in later rows, they will be erased.
- Zeroes the derivative vector.

### `copy_next_drive(iteration)`

```python
def copy_next_drive(self, iteration):
    array = self.data.network.drives.array
    array[iteration + 1] = array[iteration]
```

Copies the drive value from the current iteration to the next. This makes drives "sticky" — if no new drive is computed by `DescendingDrive.step()`, the previous iteration's drive persists. This is called at the end of every `step()`.

### `step()` — complete walkthrough

```python
def step(self, iteration, time, timestep, checks=False, strict=False):
    # Phase 0: Iteration 0 — just copy drive, no integration
    if iteration == 0:
        self.copy_next_drive(iteration)
        return

    # Phase 1: Optional consistency check
    if checks:
        assert np.array_equal(
            self.solver.y,
            np.array(self.data.state.array[iteration - 1, :]),
        ), (...)
```

**Phase 0**: On the very first iteration, no integration happens. The drive is simply copied forward. This is because the solver was already initialized with `data.state.array[0, :]` in `initialize_episode()`, so there's nothing to integrate yet — the physics hasn't produced any sensor data to feed back.

```python
    # Phase 2: Modulo skip — copy previous state if not integrating this step
    if iteration % self.modulo:
        self.data.state.array[iteration, :] = (
            self.data.state.array[iteration - 1, :]
        )
```

**Phase 2**: If `modulo > 1` and the current iteration is not a multiple of `modulo`, the previous state is copied forward without integration. This allows running the CPG controller at a lower rate than the physics simulation (e.g., integrate CPG every 2 physics steps).

```python
    # Phase 3: Integration
    else:
        self.solver.set_f_params(self.dstate, iteration, self.data)
        while self.solver.successful() and self.solver.t < time + 0.99 * timestep:
            self.data.state.array[iteration, :] = (
                self.solver.integrate(time + timestep)
            )
```

**Phase 3**: The actual integration. `set_f_params` passes `(self.dstate, iteration, self.data)` as extra arguments to the ODE function. These become the `dstate`, `iteration`, and `data` parameters of `ode_oscillators_sparse`.

The `while` loop handles sub-stepping — the integrator may need multiple internal steps to reach `time + timestep`. The condition `self.solver.t < time + 0.99 * timestep` uses `0.99` as a tolerance factor to avoid floating-point comparison issues. The loop continues until the solver has reached the target time or fails.

```python
        # Phase 4: Error handling
        if not self.solver.successful():
            message = (
                f'ODE not integrated properly at {iteration=}'
                f' ({self.solver.t=} < {time+timestep=} [s])'
                f'\nReturn code: {self.solver.get_return_code()=}'
                f'\nState:\n{np.array(self.data.state.array[iteration, :])}'
            )
            if strict:
                raise IntegrationException(message)
            pylog.warning('%s\n\nResetting to previous iteration', message)
            self.solver.set_initial_value(y=self.solver.y, t=time + timestep)
```

**Phase 4**: If integration failed:
- If `strict=True`: raises `IntegrationException` with detailed diagnostic info (current solver time, target time, return code, state vector).
- If `strict=False` (default): logs a warning and resets the solver to its current state (not the target state). This is a best-effort recovery — the simulation continues but the state may be inaccurate.

```python
    # Phase 5: Drive propagation
    if iteration < self.n_iterations - 1:
        self.copy_next_drive(iteration)

    # Phase 6: Optional post-checks
    if checks:
        assert self.solver.successful(), (...)
        assert abs(time + timestep - self.solver.t) < 1e-6 * timestep, (...)
```

**Phase 5**: The drive is copied forward (unless this is the last iteration).

**Phase 6**: If `checks=True`, verifies that the solver succeeded and that the solver time matches the expected time within `1e-6 * timestep` tolerance.

## How to integrate: adding a new sensory feedback pathway

To add a new type of sensory feedback (e.g., IMU orientation feedback to oscillator frequency):

1. **Add a new `ConnectionType` enum value** in `farms_amphibious/data/data_cy.pyx`:
   ```python
   class ConnectionType(Enum):
       ...
       ORIENTATION2FREQ = <new_value>
   ```

2. **Add a new `cdef` function** in `ode.pyx`:
   ```cython
   cpdef inline void ode_orientation(
       unsigned int iteration,
       DTYPEv1 state,
       DTYPEv1 dstate,
       ImuSensorArrayCy imu,
       ImuConnectivityCy imu2osc_map,
   ) nogil:
       cdef unsigned int i, i0, i1, connection_type
       for i in range(imu2osc_map.n_connections):
           i0 = imu2osc_map.connections.array[i, 0]
           i1 = imu2osc_map.connections.array[i, 1]
           connection_type = imu2osc_map.connections.array[i, 2]
           if connection_type == ConnectionType.ORIENTATION2FREQ:
               dstate[i0] += imu2osc_map.c_weight(i) * imu.c_pitch(iteration, i1)
   ```

3. **Call the new function** from `ode_oscillators_sparse`:
   ```cython
   ode_orientation(iteration, state, dstate, data.sensors.imu, data.network.imu2osc_map)
   ```
   Add this call after the existing sensory feedback calls, before `return dstate`.

4. **Add the connectivity data structures** to `AmphibiousDataCy` and the corresponding Python data containers.

5. **Add YAML configuration** for the new connectivity in the network options.

## How to integrate: changing the ODE function

To use a custom ODE function (e.g., a different CPG model):

```python
# Custom ODE function with the same signature
def my_custom_ode(time, state, dstate, iteration, data, nosfb=0):
    # Custom dynamics here
    dstate[:] = 0
    # ... compute derivatives ...
    return dstate

# Construct NetworkODE with custom ODE
network = NetworkODE(
    data=amphibious_data,
    integrator='dopri5',
    ode=my_custom_ode,
    nsteps=1000,
    max_step=timestep,
)
```

**Important**: The custom ODE function must accept the same arguments as `ode_oscillators_sparse`: `(time, state, dstate, iteration, data, nosfb=0)`. The `data` argument is the Python-level `AmphibiousData` object (not the Cython `AmphibiousDataCy`), so you will need to access data differently than in the Cython version.

**Note**: If your custom ODE is pure Python (not Cython), it will be significantly slower because it holds the GIL. For production use, write Cython ODE functions with `nogil`.

## How to integrate: changing the integrator

```python
# Use LSODA (automatic stiff/non-stiff switching)
network = NetworkODE(data=data, integrator='lsoda', nsteps=1000, max_step=timestep)

# Use BDF (for stiff systems)
network = NetworkODE(data=data, integrator='vode', method='bdf', nsteps=1000, max_step=timestep)
```

The integrator name and kwargs are passed directly to `scipy.integrate.ode.set_integrator()`. See [scipy's ODE documentation](https://docs.scipy.org/doc/scipy/reference/generated/scipy.integrate.ode.html) for available integrators.

## Common failure modes

### 1. Solver failure (non-strict mode)

When the solver fails and `strict=False` (the default), a warning is logged and the solver is reset to its current (possibly inaccurate) state. The simulation continues but the CPG state may be wrong. Symptoms include sudden jumps in oscillator phases or amplitudes.

**Common causes**:
- Drive values outside the valid range (drives should typically be in [0, 1] or [-1, 1])
- Numerical stiffness from very large coupling weights
- Timestep too large for the oscillator frequencies

**Fix**: Set `strict=True` in the controller construction to get an exception with diagnostic info, or reduce the `max_step` parameter.

### 2. State array corruption

The state array is pre-allocated as `data.state.array[n_iterations, n_states]`. If `n_iterations` is smaller than the actual number of simulation steps, the array will be indexed out of bounds. This manifests as a numpy IndexError.

**Fix**: Ensure `n_iterations` in the simulation options matches or exceeds the actual number of steps (`duration / timestep`).

### 3. Drive array propagation issues

`copy_next_drive` copies `drives.array[iteration]` to `drives.array[iteration + 1]`. If a `DescendingDrive` writes to the drive array at iteration `i`, but `NetworkODE.step()` is called with `iteration=i` before the drive is set, the drive from iteration `i-1` will be used instead.

**Fix**: Ensure `DescendingDrive.step()` is called BEFORE `NetworkODE.step()` in the controller's `before_step()` method. The standard `AmphibiousController.before_step()` does this in the correct order.

### 4. `dstate` not zeroed between calls

The `dstate` array is allocated once and reused. The ODE functions use `+=` to add to `dstate`. If `dstate` is not zeroed between solver calls, residual values from previous calls will accumulate. However, scipy's `ode.integrate()` calls the RHS function fresh each time, and the `dstate` is passed as an extra argument — scipy does NOT zero it.

**In practice**: This is handled correctly because `ode_oscillators_sparse` is called by scipy's internal integration loop, and the `dstate` is written (not accumulated) by the first three functions (`ode_dphase`, `ode_damplitude`, `ode_joints`) using `=` assignment, and only the sensory feedback functions use `+=`. But if you add a new core dynamics function that uses `+=` without the core functions first using `=`, you will get incorrect results.

### 5. Invalid connection types in `nogil` mode

In `nogil` mode, Python exceptions cannot be raised. The `ode_stretch` and `ode_xfrc` functions print error messages with `printf` for invalid connection types, but execution continues. This means invalid connections are **silently ignored** in production. Check stderr output for these messages during debugging.

### 6. `iteration == 0` special case

On iteration 0, `step()` returns immediately after copying the drive. This means the initial state loaded from YAML is used as-is for the first physics step. If your initial conditions need to be processed by the ODE (e.g., to compute consistent derivatives), they won't be. The first actual integration happens at iteration 1.

## What NOT to assume

1. **`nogil` does not mean thread-safe.** It means the function releases the GIL, allowing other Python threads to run. But the ODE functions are not designed for parallel execution — they share the `dstate` and `state` arrays.

2. **The `dstate` is NOT zeroed by `ode_oscillators_sparse`.** The function assumes `dstate` has been properly initialized. In practice, scipy's integrator handles this, but if you call the function directly, you must zero `dstate` first.

3. **The iteration 0 special case is intentional.** The CPG does not integrate on the first step because there is no sensor data yet (physics hasn't run). The initial state from YAML is used directly.

4. **`modulo > 1` does NOT change the timestep.** It changes how often the CPG integrates. When `modulo=2`, the CPG integrates every 2 physics steps, but the integration target time is still `time + timestep` (one physics step), not `time + 2*timestep`. The state is simply copied forward on skipped iterations.

5. **Contact feedback does NOT affect amplitude.** Only `REACTION2FREQ` and `REACTION2FREQTEGOTAE` are implemented. There is no `REACTION2AMP` connection type.

6. **`ode_xfrc` only uses the y-component of external force.** If you need x or z components, you must modify the source code. The `fabs` call means the sign is always positive.

7. **The `0.99` factor in the integration loop is a floating-point tolerance.** The condition `self.solver.t < time + 0.99 * timestep` allows the loop to exit when the solver is within 1% of the target time, avoiding infinite loops from floating-point rounding.

8. **`IntegrationException` is imported but not defined in `network.py`.** It is imported from `farms_core` (via `from farms_core import pylog`). Check `farms_core` for its definition.
