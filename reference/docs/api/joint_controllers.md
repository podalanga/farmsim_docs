# farms_amphibious.control.joints_control_cy

!!! note "Source Files"
    - `farms_amphibious/control/passive_cy.pyx` — `PassiveJointCy`
    - `farms_amphibious/control/position_muscle_cy.pyx` — `PositionMuscleCy`
    - `farms_amphibious/control/position_phase_cy.pyx` — `PositionPhaseCy`
    - `farms_amphibious/control/joints_control_cy.pyx` — Base class `JointsControlCy`

In addition to the Ekeberg torque model, FARMS provides three additional joint actuator strategies. These cover passive dynamics, position control from oscillator amplitude, and phase-tracking position control.

---

## 1. Passive Joint (`PassiveJointCy`)

A passive joint applies spring, damping, and friction torques with no active component — it behaves as a purely mechanical element.

### Torque Equation

$$
\tau = -k \cdot (\theta - b) \cdot g - d \cdot \dot{\theta} \cdot g - f \cdot \text{sign}(\dot{\theta}) \cdot g
$$

Where:

| Symbol | Parameter | Description |
|--------|-----------|-------------|
| $k$ | `stiffness_coefficient` | Spring constant (N·m/rad) |
| $d$ | `damping_coefficient` | Viscous damping (N·m·s/rad) |
| $f$ | `friction_coefficient` | Coulomb friction (N·m) |
| $b$ | `transform_bias` | Equilibrium position in sensor space |
| $g$ | `transform_gain` | Scale factor between convention and SDF space |
| $\theta$ | Joint position | From `joints_data.positions(iteration)` |
| $\dot{\theta}$ | Joint velocity | From `joints_data.velocities(iteration)` |

**From `passive_cy.pyx` (lines 50–69):**

```cython
cpdef void step(self, unsigned int iteration):
    for joint_i in range(self.n_joints):
        joint_data_i = self.indices[joint_i]
        passive_stiffness = -self.stiffness_coefficients[joint_i] * (
            positions[joint_data_i] - self.transform_bias[joint_data_i]
        ) * self.transform_gain[joint_data_i]
        damping = -(self.damping_coefficients[joint_i]
                    * velocities[joint_data_i]
                    * self.transform_gain[joint_data_i])
        friction = -(self.friction_coefficients[joint_i]
                     * sign(velocities[joint_data_i])
                     * self.transform_gain[joint_data_i])

        # Log all components
        self.joints_data.array[iteration, joint_data_i, JOINT_TORQUE]          = passive_stiffness + damping + friction
        self.joints_data.array[iteration, joint_data_i, JOINT_TORQUE_STIFFNESS]= passive_stiffness
        self.joints_data.array[iteration, joint_data_i, JOINT_TORQUE_DAMPING]  = damping
        self.joints_data.array[iteration, joint_data_i, JOINT_TORQUE_FRICTION] = friction
```

### Construction

```python
PassiveJointCy(
    stiffness_coefficients=np.array([k0, k1, ...], dtype=np.double),
    damping_coefficients=np.array([d0, d1, ...], dtype=np.double),
    friction_coefficients=np.array([f0, f1, ...], dtype=np.double),
    joints_names=['joint_leg_0_L_0', ...],
    joints_data=animat_data.sensors.joints,
    indices=passive_joints_indices,
    gain=joints_map.transform_gain,
    bias=joints_map.transform_bias,
)
```

Values come from `MotorOptions.passive.stiffness_coefficient`, `damping_coefficient`, `friction_coefficient`.

### Accessor Methods

| Method | Array Index | Description |
|--------|------------|-------------|
| `stiffness(iteration)` | `JOINT_TORQUE_STIFFNESS` | Returns logged passive stiffness |
| `damping(iteration)` | `JOINT_TORQUE_DAMPING` | Returns logged damping torque |
| `friction(iteration)` | `JOINT_TORQUE_FRICTION` | Returns logged friction torque |

---

## 2. Position Muscle (`PositionMuscleCy`)

This model drives joints in **position control mode** using the *amplitude difference* of two CPG oscillators as the target angle. It is used when MuJoCo's position servo is preferred over explicit torque control.

### Position Command Equation

$$
\theta^{cmd} = g \cdot \left(0.5 \cdot (y_1 - y_0) + \delta_j\right) + b
$$

Where:
- $y_k = r_k(1+\cos\varphi_k)$ — neural output of oscillator $k$
- $\delta_j$ — joint offset from the CPG state at `offsets(iteration)[joint_data_i]`
- $g$ — `transform_gain[joint_data_i]`
- $b$ — `transform_bias[joint_data_i]`

**From `position_muscle_cy.pyx` (lines 12–33):**

```cython
cpdef void step(self, unsigned int iteration):
    neural_activity = self.state.outputs(iteration)   # r*(1 + cos(phi)) for all oscs
    offsets = self.state.offsets(iteration)

    for joint_i in range(self.n_joints):
        joint_data_i = self.indices[joint_i]
        osc_0 = self.osc_indices[0][joint_i]
        osc_1 = self.osc_indices[1][joint_i]
        neural_diff = neural_activity[osc_1] - neural_activity[osc_0]

        self.joints_data.array[iteration, joint_data_i, JOINT_CMD_POSITION] = (
            self.transform_gain[joint_data_i]
            * (0.5 * neural_diff + offsets[joint_data_i])
            + self.transform_bias[joint_data_i]
        )
```

!!! note "Why 0.5?"
    The factor of 0.5 normalises the neural difference. Since $y_k \in [0, 2r_k]$, the difference $\Delta y \in [-2r, 2r]$. Multiplying by 0.5 gives an effective angular excursion of $r$ per side — matching the nominal amplitude parameter.

### Construction

```python
PositionMuscleCy(
    joints_names=muscles_joints,
    joints_data=animat_data.sensors.joints,
    indices=muscles_joints_indices,  # uint32 array into joint sensor data
    state=animat_data.state,         # OscillatorNetworkStateCy
    parameters=np.array(muscle_map.arrays, dtype=np.double),
    osc_indices=np.array(muscle_map.osc_indices, dtype=np.uintc),
    gain=np.array(joints_map.transform_gain, dtype=np.double),
    bias=np.array(joints_map.transform_bias, dtype=np.double),
)
```

The `position_cmds(iteration)` method (inherited from `JointsMusclesCy`) reads `JOINT_CMD_POSITION` from the array and returns it as a numpy array.

---

## 3. Position Phase (`PositionPhaseCy`)

The most sophisticated position controller. Rather than using amplitude difference, it tracks the **oscillator phase** directly, with a built-in **swim/walk switching** mechanism based on amplitude threshold.

### Two-Mode Control

```cython
cpdef void step(self, unsigned int iteration):
    offsets = self.state.offsets(iteration)
    phases = self.state.phases(iteration)
    amplitudes = self.state.amplitudes(iteration)

    for joint_i in range(self.n_joints):
        joint_data_i = self.indices[joint_i]
        pos = self.joints_data.array[iteration, joint_data_i, JOINT_POSITION]
        osc_i = self.osc_indices[0][joint_data_i]

        if amplitudes[osc_i] < self.threshold:  # Swimming (low amplitude)
            # Track theta = pi (limb retracted)
            dif = (M_PI - pos) % (2*M_PI) - M_PI
        else:  # Walking (high amplitude)
            # Track the oscillator phase
            dif = (phases[osc_i] - pos + M_PI) % (2*M_PI) - M_PI

        if dif < -M_PI:
            dif += 2*M_PI

        self.joints_data.array[iteration, joint_data_i, JOINT_CMD_POSITION] = (
            self.transform_gain[joint_data_i] * (dif + pos + offsets[joint_data_i])
            + self.transform_bias[joint_data_i]
        )
```

### Modes Explained

**Swimming mode (`amplitude < threshold`):**

$$
\Delta\theta = \frac{(\pi - \theta) \mod 2\pi - \pi}{1}
$$

$$
\theta^{cmd} = g \cdot (\Delta\theta + \theta + \delta_j) + b
$$

The limb is driven toward $\theta = \pi$ (retracted position), effectively feathering the legs to reduce drag while the body swims with undulatory motion.

**Walking mode (`amplitude >= threshold`):**

$$
\Delta\theta = \frac{(\varphi_i - \theta) \mod 2\pi - \pi}{1}
$$

$$
\theta^{cmd} = g \cdot (\Delta\theta + \theta + \delta_j) + b
$$

The joint position tracks the oscillator phase directly — the angular difference `dif` acts as a proportional correction signal, effectively implementing phase-error-based position control with gain proportional to `transform_gain`.

!!! important "The threshold as a gait switch"
    The `threshold` parameter (default `1e-2`) compares against the oscillator amplitude. When the descending drive is low (swimming regime), the nominal amplitude converges to near zero, which drops below threshold and triggers retraction. When drive increases (walking regime), amplitude rises above threshold, enabling phase tracking. This creates an automatic gait transition without explicit state machines.

### Constructor Parameters

| Parameter | Type | Description |
|---|---|---|
| `state` | `OscillatorNetworkStateCy` | CPG state array |
| `osc_indices` | `uint32[2, N_joints]` | Oscillator index per joint (uses `[0]` index) |
| `weight` | `float` | (unused in current code, reserved) |
| `offset` | `float` | Phase offset (`0.25*π` by default in `AmphibiousController`) |
| `threshold` | `float` | Amplitude threshold for walk/swim switching (`1e-2` default) |

**From `amphibious.py` (lines 455–466):**

```python
self.network2joints['phase'] = PositionPhaseCy(
    joints_names=muscles_joints,
    joints_data=self.animat_data.sensors.joints,
    indices=muscles_joints_indices,
    state=self.animat_data.state,
    osc_indices=np.array(muscle_map.osc_indices, dtype=np.uintc),
    gain=np.array(self.joints_map.transform_gain, dtype=np.double),
    bias=np.array(self.joints_map.transform_bias, dtype=np.double),
    weight=-1e6,         # High-gain, not used in step
    offset=0.25*np.pi,   # Quarter-turn phase offset
    threshold=1e-2,      # Amplitude below this → swimming mode (legs retracted)
)
```

---

## Base Class: `JointsControlCy`

All joint controller Cython classes inherit from `JointsControlCy` (defined in `joints_control_cy.pxd`):

| Attribute | Type | Description |
|---|---|---|
| `n_joints` | `int` | Number of controlled joints |
| `indices` | `uint32[N_joints]` | Indices into `joints_data` array |
| `joints_data` | `JointSensorArrayCy` | Shared sensor/command array |
| `joints_names` | `list[str]` | Joint names for output dict construction |
| `transform_gain` | `float64[N_joints_total]` | Convention-space to SDF-space gain |
| `transform_bias` | `float64[N_joints_total]` | Convention-space to SDF-space bias |

The `transform_gain` and `transform_bias` allow a joint defined with range `[-π, π]` in the SDF to be controlled in a `[-1, 1]` convention space, or to map a flipped joint axis, for example.

---

## Controller Selection: `equations_dict`

In `JointMuscleController.__init__`, each motor's equation determines which Cython object gets instantiated:

```python
self.equations_dict = {
    motor.joint_name: motor.equation   # from MotorOptions
    for motor in animat_options.control.motors
}
```

| `motor.equation` | Class Instantiated | Control Type |
|---|---|---|
| `'ekeberg_muscle'` | `EkebergMuscleCy` | Torque (implicit spring) |
| `'ekeberg_muscle_explicit'` | `EkebergMuscleCy` | Torque (explicit, no MuJoCo spring) |
| `'passive'` | `PassiveJointCy` | Torque (passive only) |
| `'position_muscle'` | `PositionMuscleCy` | Position (amplitude-diff based) |
| `'phase'` | `PositionPhaseCy` | Position (phase tracking + gait switching) |

Multiple equation types can coexist in one controller. The `before_step` loop calls `net2joints.step(index)` for each:

```python
def before_step(self, task, action, physics):
    index = task.iteration % task.buffer_size
    self.network.step(index=index, time=..., timestep=...)
    for net2joints in self.network2joints.values():
        net2joints.step(index)   # EkebergMuscleCy, PassiveJointCy, etc.
```

---

## See Also

- [Ekeberg Muscle Model](ekeberg_muscle.md) — Deep dive into active muscle torques
- [CPG Oscillators](cpg_oscillators.md) — Source of all `outputs()` and `phases()` used here
- [Amphibious Controller](farms_amphibious_controller.md) — How all controllers are assembled
