# Cython Equation Handlers

This page documents the Cython classes that convert CPG oscillator state into joint-level motor commands. Each handler implements a different muscle/joint model and is selected per-joint via the `equation` field in the animat's motor configuration.

## Source files covered

| File | Lines | Purpose |
|---|---|---|
| `farms_amphibious/control/position_phase_cy.pyx` | 67 | `PositionPhaseCy` — phase-based position control |
| `farms_amphibious/control/ekeberg.pyx` | 131 | `EkebergMuscleCy` — Ekeberg muscle model |
| `farms_amphibious/control/amphibious.py` | 695 | `JointMuscleController`, `AmphibiousController`, `JointsMap`, `MusclesMap` |
| `farms_amphibious/control/passive_cy.pyx` | — | `PassiveJointCy` — passive stiffness/damping |
| `farms_amphibious/control/position_muscle_cy.pyx` | — | `PositionMuscleCy` — amplitude-based position control |
| `farms_amphibious/control/joints_control_cy.pyx` | — | `JointsControlCy`, `JointsMusclesCy` (base classes) |

## Call graph / entry points

```
ExperimentTask.before_step()
  └─ AmphibiousController.before_step()
       ├─ drive.step(iteration, time, timestep)        [DescendingDrive]
       ├─ network.step(iteration, time, timestep)       [NetworkODE — integrate CPG]
       └─ for net2joints in self.network2joints.values():
            net2joints.step(iteration)                  [Cython handler — compute motor commands]

ExperimentTask.step_joints_control_position/torque()
  └─ controller.positions/torques(iteration, time, timestep)
       └─ for equation in self.equations[ControlType.X]:
            equation(iteration, time, timestep)         [Python wrapper → reads Cython results]
```

## Class hierarchy

```
JointsControlCy (Cython base class)
  ├── PassiveJointCy        — passive stiffness/damping/friction
  └── JointsMusclesCy (extends JointsControlCy)
       ├── EkebergMuscleCy   — Ekeberg muscle model (5-parameter)
       └── PositionMuscleCy  — amplitude-based position

PositionPhaseCy (extends JointsControlCy)
  — Phase-based position control (separate from JointsMusclesCy)
```

## Selection mechanism

In `JointMuscleController.__init__`, the `equation` field from each motor's options determines which handler is instantiated:

```python
self.equations_dict = {
    motor.joint_name: motor.equation
    for motor in animat_options.control.motors
}
```

The equation types and their handlers:

| Equation string | Handler class | Control type | Description |
|---|---|---|---|
| `'ekeberg_muscle'` | `EkebergMuscleCy` | TORQUE | Ekeberg muscle model with implicit passive dynamics |
| `'ekeberg_muscle_explicit'` | `EkebergMuscleCy` | TORQUE | Ekeberg muscle model with explicit passive dynamics |
| `'passive'` | `PassiveJointCy` | TORQUE | Passive joint with stiffness/damping/friction |
| `'position_muscle'` | `PositionMuscleCy` | POSITION | Amplitude-based position control |
| `'position_phase'` | `PositionPhaseCy` | POSITION | Phase-based position control |

A single animat can have different equation types for different joints. The `equations_dict` maps joint names to equation strings, and the controller creates separate `network2joints` handlers for each equation type.

## `JointsMap`

```python
class JointsMap:
    def __init__(self, joints, joints_sensors_names, animat_options):
        control_types = list(ControlType)
        self.indices = [
            np.array([
                joint_i
                for joint_i, joint in enumerate(joints_sensors_names)
                if joint in joints[control_type]
            ])
            for control_type in control_types
        ]
        transform_gains = {
            motor.joint_name: motor.transform.gain
            for motor in animat_options.control.motors
        }
        self.transform_gain = np.array([
            transform_gains[joint]
            for joint in joints_sensors_names
        ])
        transform_bias = {
            motor.joint_name: motor.transform.bias
            for motor in animat_options.control.motors
        }
        self.transform_bias = np.array([
            transform_bias[joint]
            for joint in joints_sensors_names
        ])
```

### Purpose

Maps between joint names in the sensor data array and control-type-specific joint indices. Also provides per-joint transform parameters.

### Attributes

| Attribute | Type | Description |
|---|---|---|
| `indices` | list[np.ndarray] | Per-control-type arrays of sensor data indices for joints of that type |
| `transform_gain` | np.ndarray | Per-sensor-joint gain values for SDF↔convention space conversion |
| `transform_bias` | np.ndarray | Per-sensor-joint bias values for SDF↔convention space conversion |

### Transform parameters

The `transform_gain` and `transform_bias` arrays convert between SDF joint space and the amphibious convention space:

```
convention_angle = (sdf_angle - transform_bias) / transform_gain
sdf_angle = convention_angle * transform_gain + transform_bias
```

This is used by all Cython handlers to read joint positions in convention space and write commands back in SDF space.

## `MusclesMap`

```python
class MusclesMap:
    def __init__(self, joints, animat_options, animat_data):
        joint_muscle_map = {
            muscle.joint_name: muscle
            for muscle in animat_options.control.muscles
        }
        muscles = [joint_muscle_map[joint] for joint in joints]
        self.arrays = np.array([
            [muscle.alpha, muscle.beta, muscle.gamma, muscle.delta, muscle.epsilon]
            for muscle in muscles
        ], dtype=np.double)
        if animat_data.network is None:
            self.osc_indices = np.array([[], []], dtype=np.uintc)
            return
        osc_names = animat_data.network.oscillators.names
        self.osc_indices = np.array([
            [osc_names.index(muscle.osc1) if muscle.osc1 in osc_names else np.iinfo(np.uintc).max
             for muscle in muscles],
            [osc_names.index(muscle.osc2) if muscle.osc2 in osc_names else np.iinfo(np.uintc).max
             for muscle in muscles],
        ], dtype=np.uintc)
```

### Purpose

Maps muscle parameters and oscillator indices for each joint that uses a muscle-based equation.

### Attributes

| Attribute | Type | Shape | Description |
|---|---|---|---|
| `arrays` | np.ndarray | `[n_joints, 5]` | Muscle parameters: alpha, beta, gamma, delta, epsilon |
| `osc_indices` | np.ndarray | `[2, n_joints]` | Oscillator indices for the two opposing muscles per joint |

### Oscillator indices

Each joint with a muscle-based equation has TWO oscillators (opposing muscles): `osc1` and `osc2`. The `osc_indices` array has shape `[2, n_joints]`:
- `osc_indices[0][i]`: Index of the first oscillator (muscle) for joint `i`
- `osc_indices[1][i]`: Index of the second oscillator (muscle) for joint `i`

If an oscillator name is not found in `osc_names`, the index is set to `np.iinfo(np.uintc).max` (maximum unsigned int) as a sentinel value.

### Muscle parameters

The 5 parameters (alpha, beta, gamma, delta, epsilon) correspond to the Ekeberg muscle model:

| Parameter | Symbol | Role in Ekeberg model |
|---|---|---|
| `alpha` | α | Active torque gain (neural difference → torque) |
| `beta` | β | Active stiffness gain (neural sum × position error → stiffness) |
| `gamma` | γ | Passive stiffness gain |
| `delta` | δ | Damping coefficient (velocity → damping torque) |
| `epsilon` | ε | Friction coefficient (velocity sign → friction torque) |

## `PositionPhaseCy`

```python
cdef class PositionPhaseCy(JointsControlCy):
    def __init__(self, OscillatorNetworkStateCy state, UITYPEv2 osc_indices, **kwargs):
        self.state = state
        self.osc_indices = osc_indices
        self.threshold = kwargs.pop('threshold', 0)
        super().__init__(**kwargs)
```

### Constructor parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `state` | `OscillatorNetworkStateCy` | required | CPG state array (phases, amplitudes, offsets) |
| `osc_indices` | `UITYPEv2` (uint array) | required | Oscillator indices per joint `[2, n_joints]` |
| `threshold` | float | 0 | Amplitude threshold for swim/walk gait switching |
| `**kwargs` | — | — | Passed to `JointsControlCy`: `joints_names`, `joints_data`, `indices`, `gain`, `bias` |

### `step(iteration)` — complete walkthrough

```cython
cpdef void step(self, unsigned int iteration):
    cdef double pos, pos_raw, dif
    cdef unsigned int joint_i, joint_data_i, osc_i_0, osc_i_1
    cdef DTYPEv1 offsets = self.state.offsets(iteration)
    cdef DTYPEv1 phases = self.state.phases(iteration)
    cdef DTYPEv1 amplitudes = self.state.amplitudes(iteration)

    for joint_i in range(self.n_joints):
        # 1. Read current joint position and transform to convention space
        joint_data_i = self.indices[joint_i]
        pos_raw = self.joints_data.array[iteration, joint_data_i, JOINT_POSITION]
        pos = (pos_raw - self.transform_bias[joint_data_i]) / self.transform_gain[joint_data_i]

        # 2. Get oscillator indices for this joint
        osc_i_0 = self.osc_indices[0][joint_i]
        osc_i_1 = self.osc_indices[1][joint_i]

        # 3. Assertions
        assert osc_i_0 < len(phases), ...
        assert osc_i_1 >= len(phases), ...  # Second osc should be out of range

        # 4. Gait selection: swim vs walk
        if amplitudes[osc_i_0] < self.threshold:  # Swimming
            desired_angle = 0 + offsets[joint_data_i]
        else:  # Walking
            desired_angle = phases[osc_i_0] + offsets[joint_data_i]

        # 5. Compute shortest angular distance
        dif = fmod(desired_angle - pos + M_PI, 2*M_PI) - M_PI
        if dif < -M_PI:
            dif += 2*M_PI

        # 6. Write command in SDF space
        self.joints_data.array[iteration, joint_data_i, JOINT_CMD_POSITION] = (
            self.transform_gain[joint_data_i] * (dif + pos)
            + self.transform_bias[joint_data_i]
        )
```

### Gait switching logic

The `threshold` parameter (set to `1e-2` in `AmphibiousController.__init__`) determines the swim/walk transition:

- **Swimming** (`amplitudes[osc_i_0] < threshold`): The desired angle is just the joint offset. The phase is NOT used — the joint holds a static position. This is because swimming uses axial undulation controlled by other joints, and the limbs stay retracted.

- **Walking** (`amplitudes[osc_i_0] >= threshold`): The desired angle is the oscillator phase PLUS the joint offset. The phase drives the oscillatory motion of the limb.

### Shortest angular distance computation

```cython
dif = fmod(desired_angle - pos + M_PI, 2*M_PI) - M_PI
if dif < -M_PI:
    dif += 2*M_PI
```

This computes the shortest angular difference between the desired angle and the current position, wrapped to \([-\pi, \pi]\). The `fmod` call computes the modulo, and the `if` handles the edge case where `fmod` returns a value less than \(-\pi\).

### Assertion on oscillator indices

```cython
assert osc_i_0 < len(phases)
assert osc_i_1 >= len(phases)
```

The first assertion checks that the primary oscillator index is valid. The second assertion checks that the secondary oscillator index is **out of range** — this is intentional. `PositionPhaseCy` uses only ONE oscillator per joint (the phase oscillator), not a pair. The second index should be the sentinel value `np.iinfo(np.uintc).max`.

### Final position command

```cython
self.joints_data.array[iteration, joint_data_i, JOINT_CMD_POSITION] = (
    self.transform_gain[joint_data_i] * (dif + pos)
    + self.transform_bias[joint_data_i]
)
```

The command is the current position PLUS the shortest angular difference, transformed back to SDF space. This is a position command: it tells the position actuator to move to this angle.

## `EkebergMuscleCy`

```python
cdef class EkebergMuscleCy(JointsMusclesCy):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.activations = np.zeros(self.n_joints, dtype=np.double)
        self.joints_offsets = np.zeros(self.n_joints, dtype=np.double)
        self.spring_coefs = np.zeros(self.n_joints, dtype=np.double)
        self.damping_coefs = np.zeros(self.n_joints, dtype=np.double)
```

### Internal state

| Attribute | Type | Description |
|---|---|---|
| `activations` | np.ndarray | Neural output per oscillator (from `state.outputs()`) |
| `joints_offsets` | np.ndarray | Joint offset values (from `state.offsets()`) |
| `spring_coefs` | np.ndarray | Computed spring coefficients (for MuJoCo spring ref) |
| `damping_coefs` | np.ndarray | Computed damping coefficients (for MuJoCo damping) |

### Parameter enum

```cython
cdef enum:
    ALPHA = 0
    BETA = 1
    GAMMA = 2
    DELTA = 3
    EPSILON = 4
```

These are the indices into the `parameters` array (from `MusclesMap.arrays`).

### `step(iteration)` — complete walkthrough

```cython
cpdef void step(self, unsigned int iteration):
    cdef unsigned int muscle_i, joint_data_i, osc_0, osc_1
    cdef DTYPE neural_diff, neural_sum
    cdef DTYPE active_torque, stiffness_intermediate
    cdef DTYPE active_stiffness, passive_stiffness, damping, friction
    cdef DTYPEv1 positions = self.joints_data.positions(iteration)
    cdef DTYPEv1 velocities = self.joints_data.velocities(iteration)

    # Update activations and offsets from CPG state
    self.update_activations(iteration)  # self.activations = self.state.outputs(iteration)
    self.update_offsets(iteration)      # self.joints_offsets = self.state.offsets(iteration)

    for muscle_i in range(self.n_joints):
        joint_data_i = self.indices[muscle_i]
        osc_0 = self.osc_indices[0][muscle_i]
        osc_1 = self.osc_indices[1][muscle_i]

        # 1. Neural signals
        neural_diff = self.activations[osc_1] - self.activations[osc_0]
        neural_sum = self.activations[osc_0] + self.activations[osc_1]

        # 2. Position error in convention space
        m_delta_phi = self.joints_offsets[muscle_i] - (
            positions[joint_data_i] - self.transform_bias[joint_data_i]
        ) / self.transform_gain[joint_data_i]

        # 3. Torque components (5 total)
        active_torque = alpha * neural_diff * transform_gain       # SDF space
        stiffness_intermediate = beta * m_delta_phi
        active_stiffness = neural_sum * stiffness_intermediate * transform_gain
        passive_stiffness = gamma * stiffness_intermediate * transform_gain
        damping = -(delta * velocities[joint_data_i])
        friction = -(epsilon * sign(velocities[joint_data_i]))

        # 4. Store coefficients for MuJoCo spring/damping
        self.damping_coefs[muscle_i] = delta
        self.spring_coefs[muscle_i] = beta * (neural_sum + gamma)

        # 5. Transform offset to SDF space
        self.joints_offsets[muscle_i] = transform_gain * offsets + transform_bias

        # 6. Log results
        torque = active_torque + active_stiffness + passive_stiffness + damping + friction
        self.joints_data.array[iteration, joint_data_i, JOINT_CMD_TORQUE] = torque
        self.joints_data.array[iteration, joint_data_i, JOINT_TORQUE_ACTIVE] = active_torque
        self.joints_data.array[iteration, joint_data_i, JOINT_TORQUE_STIFFNESS] = passive_stiffness
        self.joints_data.array[iteration, joint_data_i, JOINT_TORQUE_DAMPING] = damping
        self.joints_data.array[iteration, joint_data_i, JOINT_TORQUE_FRICTION] = friction
```

### Neural signals

- `neural_diff = activations[osc_1] - activations[osc_0]`: The difference between the two opposing muscle activations. This drives the active torque — when one muscle is more active than the other, it creates a net torque.

- `neural_sum = activations[osc_0] + activations[osc_1]`: The total activation. This drives the active stiffness — co-contraction of both muscles increases joint stiffness without changing the net torque.

### Position error

```cython
m_delta_phi = self.joints_offsets[muscle_i] - (
    positions[joint_data_i] - self.transform_bias[joint_data_i]
) / self.transform_gain[joint_data_i]
```

The position error is computed in **convention space** (not SDF space). The current joint position is transformed from SDF to convention space, then subtracted from the desired offset. This error drives the stiffness terms.

### Five torque components

| Component | Formula | Description |
|---|---|---|
| `active_torque` | `α × neural_diff × gain` | Net torque from muscle imbalance |
| `active_stiffness` | `neural_sum × β × m_delta_phi × gain` | Position-dependent stiffness from co-contraction |
| `passive_stiffness` | `γ × β × m_delta_phi × gain` | Passive position-dependent stiffness |
| `damping` | `-δ × velocity` | Velocity-dependent damping |
| `friction` | `-ε × sign(velocity)` | Coulomb friction |

**Total torque**: `active_torque + active_stiffness + passive_stiffness + damping + friction`

Note: The `active_stiffness` is stored in `JOINT_TORQUE_ACTIVE` (commented out in the code: `#  + active_stiffness`), but only `active_torque` is actually logged there. The `active_stiffness` is part of the total torque but logged separately.

### Spring and damping coefficients

```cython
self.damping_coefs[muscle_i] = delta
self.spring_coefs[muscle_i] = beta * (neural_sum + gamma)
```

These are stored for use by `AmphibiousController.springcoefs()` and `AmphibiousController.dampingcoefs()`, which write them to MuJoCo's `jnt_stiffness` and `dof_damping` model arrays. The spring coefficient combines active (neural_sum × beta) and passive (gamma × beta) stiffness.

### Offset transform to SDF space

```cython
self.joints_offsets[muscle_i] = (
    self.transform_gain[joint_data_i] * self.joints_offsets[muscle_i]
    + self.transform_bias[joint_data_i]
)
```

After computation, the joint offset is transformed back to SDF space. This is used by `AmphibiousController.springrefs()` which writes it to MuJoCo's `qpos_spring` (spring reference position).

### `sign()` helper

```cython
cdef inline double sign(double value):
    if value < 0:
        return -1
    else:
        return 1
```

Note: `sign(0)` returns `1`, not `0`. This is a slight asymmetry — at zero velocity, friction is positive. This differs from `np.sign(0) = 0`.

## `PassiveJointCy`

Handles passive joints with stiffness, damping, and friction. Created when `motor.equation == 'passive'` in the motor options.

### Constructor parameters

| Parameter | Description |
|---|---|
| `stiffness_coefficients` | Per-joint stiffness coefficient |
| `damping_coefficients` | Per-joint damping coefficient |
| `friction_coefficients` | Per-joint friction coefficient |
| `joints_names` | Joint names |
| `joints_data` | Joint sensor data |
| `indices` | Sensor data indices |
| `gain` | Transform gain (SDF↔convention) |
| `bias` | Transform bias (SDF↔convention) |

The passive joint model computes: `torque = stiffness × (reference - position) + damping × (-velocity) + friction × (-sign(velocity))`.

## Controller integration

### `JointMuscleController.before_step()`

```python
def before_step(self, task, action, physics):
    del action
    index = task.iteration % task.buffer_size
    self.network.step(
        index=index,
        time=physics.time()/task.units.seconds,
        timestep=physics.timestep()/task.units.seconds,
    )
    for net2joints in self.network2joints.values():
        net2joints.step(index)
```

The flow is:
1. Integrate the CPG network forward one step.
2. For each Cython handler, compute motor commands from the updated CPG state.

### `AmphibiousController.step()` (extended)

```python
def step(self, iteration, time, timestep):
    if self.drive is not None:
        self.drive.step(iteration, time, timestep)
    if self.network is not None:
        self.network.step(iteration, time, timestep)
    for net2joints in self.network2joints.values():
        if net2joints is not None:
            net2joints.step(iteration)
```

The order is critical: **drive → network → handlers**. The drive sets the drive signal, the network integrates the CPG using that drive, and the handlers compute motor commands from the CPG state.

### How equations_dict selects handlers

```python
# Ekeberg muscle
for torque_equation in ['ekeberg_muscle', 'ekeberg_muscle_explicit']:
    if torque_equation not in self.equations_dict.values():
        continue
    # ... create EkebergMuscleCy ...
    self.equations[ControlType.TORQUE] += [{...}[torque_equation]]

# Passive
if 'passive' in self.equations_dict.values():
    self.equations[ControlType.TORQUE] += [self.passive]
    # ... create PassiveJointCy ...

# Position muscle (in AmphibiousController)
if 'position_muscle' in self.equations_dict.values():
    self.equations[ControlType.POSITION] += [self.positions_network]
    # ... create PositionMuscleCy ...

# Position phase (in AmphibiousController)
if 'position_phase' in self.equations_dict.values():
    self.equations[ControlType.POSITION] += [self.phases_network]
    # ... create PositionPhaseCy ...
```

The `equations` tuple has three lists: `[POSITION_equations, VELOCITY_equations, TORQUE_equations]`. Each is a list of Python callable wrappers that read the Cython handler results.

## How to integrate: adding a new equation type

1. **Create a Cython handler** (e.g., `my_equation_cy.pyx`):

```cython
cdef class MyEquationCy(JointsControlCy):
    def __init__(self, state, **kwargs):
        self.state = state
        super().__init__(**kwargs)

    cpdef void step(self, unsigned int iteration):
        for joint_i in range(self.n_joints):
            joint_data_i = self.indices[joint_i]
            # ... compute motor command ...
            self.joints_data.array[iteration, joint_data_i, JOINT_CMD_POSITION] = command
```

2. **Register the equation** in `JointMuscleController.__init__` or `AmphibiousController.__init__`:

```python
if 'my_equation' in self.equations_dict.values():
    self.equations[ControlType.POSITION] += [self.my_equation_method]
    my_joints = [m.joint_name for m in animat_options.control.motors if m.equation == 'my_equation']
    self.network2joints['my_equation'] = MyEquationCy(
        joints_names=my_joints,
        joints_data=self.animat_data.sensors.joints,
        indices=...,
        state=self.animat_data.state,
        ...
    )
```

3. **Add the method** that reads the Cython results:

```python
def my_equation_method(self, iteration, time, timestep):
    return dict(zip(
        self.network2joints['my_equation'].joints_names,
        self.network2joints['my_equation'].position_cmds(iteration),
    ))
```

4. **Add the equation string** to the YAML motor configuration:

```yaml
control:
  motors:
    - joint_name: joint_body_5
      equation: my_equation
      control_types: [position]
```

## How to integrate: changing motor equation for a specific joint

In the animat's YAML configuration:

```yaml
control:
  motors:
    - joint_name: joint_body_0
      equation: position_phase      # Phase-based position control
      control_types: [position]
    - joint_name: joint_body_1
      equation: ekeberg_muscle     # Ekeberg muscle model
      control_types: [torque, velocity]
    - joint_name: joint_passive_0
      equation: passive            # Passive joint
      control_types: [torque]
```

Each joint can have a different equation. The `control_types` list determines which MuJoCo actuators are created for that joint.

## Common failure modes

### 1. Oscillator index mismatches

If `MusclesMap.osc_indices` references oscillator names that don't exist in `animat_data.network.oscillators.names`, the index is set to `np.iinfo(np.uintc).max`. This will cause out-of-bounds array access in `EkebergMuscleCy.step()` when accessing `self.activations[osc_0]`.

**Fix**: Ensure muscle YAML `osc1` and `osc2` names match oscillator names in the network YAML.

### 2. Transform gain/bias errors

If the `transform.gain` and `transform.bias` in the motor YAML don't match the SDF joint convention, the Cython handlers will compute incorrect positions and commands. Symptoms: joints move to wrong angles or oscillate around the wrong center.

**Fix**: Verify that `convention_angle = (sdf_angle - bias) / gain` produces the expected convention-space angle. For most joints, gain=1 and bias=0.

### 3. Threshold for gait switching

The `threshold` parameter in `PositionPhaseCy` (set to `1e-2`) determines when a joint switches from swimming to walking mode. If the CPG amplitudes don't cross this threshold, the joint stays in swimming mode (no phase-driven motion).

**Fix**: Adjust the threshold or check that the CPG amplitudes are reaching the expected values.

### 4. `sign(0) = 1` in Ekeberg model

The `sign()` function returns 1 for zero velocity, not 0. This means friction is always applied, even when the joint is stationary. At zero velocity, the friction force is `epsilon` (positive), which can cause a small persistent torque.

### 5. Missing `farms_muscle` for rigid tendon muscles

If `farms_muscle` is not installed, the MuJoCo muscle callbacks are not set. This affects any actuators that use MuJoCo's built-in muscle model, but NOT the Cython handlers (which compute their own torques).

## What NOT to assume

1. **The 0.5 normalization factor** is NOT present in the Ekeberg model. The neural output from `state.outputs()` is `0.5 * amplitude * (1 + cos(phase))` (computed elsewhere), so by the time it reaches `EkebergMuscleCy`, it's already normalized.

2. **The `threshold` default is NOT 0 in practice.** Although the constructor defaults to 0, `AmphibiousController.__init__` explicitly passes `threshold=1e-2`.

3. **`PositionPhaseCy` uses only ONE oscillator per joint.** The second oscillator index should be the sentinel value `max_uint`. The assertion `assert osc_i_1 >= len(phases)` verifies this.

4. **`EkebergMuscleCy` writes FIVE torque components** but only the total is used for the torque command. The individual components are logged for debugging/analysis.

5. **The `equations` tuple is `[POSITION, VELOCITY, TORQUE]`.** The order corresponds to `ControlType` enum values (POSITION=0, VELOCITY=1, TORQUE=2). Do not change this order.

6. **`passive` joints do NOT have oscillators.** They are controlled purely by stiffness, damping, and friction. They appear in the torque control path but not in the CPG network.
