# farms_amphibious.control.ekeberg

!!! note "Source Files"
    - `farms_amphibious/control/ekeberg.pyx` — Cython implementation (compiled to `.pyd`)
    - `farms_amphibious/control/ekeberg.pxd` — Cython declarations
    - `farms_amphibious/control/amphibious.py` — `MusclesMap` constructor

The **Ekeberg Muscle Model** is FARMS' primary actuator model for joints with CPG-driven, antagonistic muscle pairs. Based on Ekeberg (1993), it models each joint as having two opposing "muscles" (flexor and extensor), each driven by a separate oscillator's neural output.

---

## Biological Motivation

In vertebrate locomotion, joints are actuated by antagonistic muscle pairs. Each muscle is activated by motor neurons whose firing rate follows the CPG rhythm. Ekeberg's model captures three physical effects in a single torque equation:

1. **Active torque** — proportional to the *difference* in neural activation (net muscle pull)
2. **Active stiffness** — proportional to the *sum* of neural activations (co-contraction stiffness)
3. **Passive stiffness** — a baseline spring from the joint mechanics
4. **Viscous damping** — velocity-proportional resistance
5. **Coulomb friction** — velocity-sign-proportional resistance

---

## The Torque Equation

Given flexor oscillator index `osc_0` and extensor oscillator index `osc_1`, define:

$$
y_k = r_k(1 + \cos(\varphi_k)) \quad \text{(neural output)}
$$

$$
\Delta y = y_1 - y_0 \quad \text{(neural difference)}
$$

$$
\Sigma y = y_0 + y_1 \quad \text{(neural sum)}
$$

$$
\Delta\phi = \delta_j - \frac{\theta - b}{g} \quad \text{(position error in convention space)}
$$

Where:
- $\delta_j$ — joint offset (from CPG state `offsets(iteration)`)
- $\theta$ — current joint position (sensor reading)
- $g$ — transform gain (`motor.transform.gain`)
- $b$ — transform bias (`motor.transform.bias`)

The total torque is:

$$
\tau = \underbrace{\alpha \cdot \Delta y \cdot g}_{\text{active torque}}
+ \underbrace{\Sigma y \cdot \beta \cdot \Delta\phi \cdot g}_{\text{active stiffness}}
+ \underbrace{\gamma \cdot \beta \cdot \Delta\phi \cdot g}_{\text{passive stiffness}}
- \underbrace{\delta \cdot \dot{\theta}}_{\text{damping}}
- \underbrace{\varepsilon \cdot \text{sign}(\dot{\theta})}_{\text{friction}}
$$

**From `ekeberg.pyx` (lines 43–131):**

```cython
cpdef void step(self, unsigned int iteration):
    for muscle_i in range(self.n_joints):
        joint_data_i = self.indices[muscle_i]
        osc_0 = self.osc_indices[0][muscle_i]
        osc_1 = self.osc_indices[1][muscle_i]

        # Neural outputs (flexor, extensor)
        neural_diff = self.activations[osc_1] - self.activations[osc_0]
        neural_sum  = self.activations[osc_0] + self.activations[osc_1]

        # Position error in convention space
        m_delta_phi = self.joints_offsets[muscle_i] - (
            positions[joint_data_i] - self.transform_bias[joint_data_i]
        ) / self.transform_gain[joint_data_i]

        # Torque components
        active_torque    = parameters[ALPHA] * neural_diff * gain
        stiffness_inter  = parameters[BETA] * m_delta_phi
        active_stiffness = neural_sum * stiffness_inter * gain
        passive_stiffness= parameters[GAMMA] * stiffness_inter * gain
        damping          = -(parameters[DELTA] * velocities[joint_data_i])
        friction         = -(parameters[EPSILON] * sign(velocities[joint_data_i]))

        torque = active_torque + active_stiffness + passive_stiffness + damping + friction
```

---

## Parameters: α, β, γ, δ, ε

| Enum | Parameter | Physics Role | Typical Range |
|------|-----------|-------------|---------------|
| `ALPHA=0` | α | Active torque gain | 1–100 N·m |
| `BETA=1` | β | Stiffness coefficient | 1–50 N·m/rad |
| `GAMMA=2` | γ | Passive stiffness ratio | 0.1–5 |
| `DELTA=3` | δ | Viscous damping | 0.01–5 N·m·s/rad |
| `EPSILON=4` | ε | Coulomb friction | 0–2 N·m |

These are set in `AmphibiousMuscleSetOptions` per joint, and assembled in `MusclesMap`:

```python
class MusclesMap:
    def __init__(self, joints, animat_options, animat_data):
        muscles = [joint_muscle_map[joint] for joint in joints]
        self.arrays = np.array([
            [m.alpha, m.beta, m.gamma, m.delta, m.epsilon]
            for m in muscles
        ], dtype=np.double)
        # Oscillator index pairs [flexor, extensor] for each joint
        osc_names = animat_data.network.oscillators.names
        self.osc_indices = np.array([
            [osc_names.index(m.osc1) for m in muscles],  # flexor
            [osc_names.index(m.osc2) for m in muscles],  # extensor
        ], dtype=np.uintc)
```

---

## Spring and Damping Coefficients for MuJoCo

The Ekeberg model also provides **live stiffness and damping coefficients** for MuJoCo's internal spring model. This allows the physics engine to handle the implicit passive dynamics more stably:

$$
k_{spring} = \beta \cdot (\Sigma y + \gamma)
$$

$$
k_{damp} = \delta
$$

**From `ekeberg.pyx` (lines 111–115):**

```cython
self.damping_coefs[muscle_i] = self.parameters[muscle_i][DELTA]
self.spring_coefs[muscle_i] = self.parameters[muscle_i][BETA] * (
    neural_sum + self.parameters[muscle_i][GAMMA]
)
```

These are returned by `springcoefs()` and `dampingcoefs()` in `JointMuscleController`, and written directly to `physics.model.jnt_stiffness` and `physics.model.dof_damping` in `ExperimentTask.step_joints_control_torque()`.

This is a key architectural detail: the **active torque command is sent through `physics.data.ctrl`**, while the **spring/damping model parameters are written to `physics.model`** (the static MuJoCo model struct). This allows MuJoCo to implicitly solve the passive dynamics at sub-step level.

---

## Joint Offset (Spring Reference): `joints_offsets`

The spring reference position (equilibrium) passed to MuJoCo is the Ekeberg joint offset transformed back to SDF/MuJoCo coordinate space:

$$
\theta^{ref} = g \cdot \delta_j + b
$$

**From `ekeberg.pyx` (lines 117–122):**

```cython
self.joints_offsets[muscle_i] = (
    self.transform_gain[joint_data_i] * self.joints_offsets[muscle_i]
    + self.transform_bias[joint_data_i]
)
```

This is then returned by `springrefs()` and written to `physics.model.qpos_spring`.

---

## `ekeberg_muscle` vs `ekeberg_muscle_explicit`

Two equation modes exist:

| Mode | Description | `torques_implicit` vs `torque_cmds` |
|------|-------------|-------------------------------------|
| `ekeberg_muscle` | Torque + spring model in MuJoCo | Uses `torques_implicit(iteration)` — provides torque *and* updates spring params |
| `ekeberg_muscle_explicit` | Pure torque output, no MuJoCo spring model | Uses `torque_cmds(iteration)` — returns raw computed torque |

`ekeberg_muscle` is preferred when MuJoCo's built-in spring/damping solver improves stability (e.g., faster contacts). `ekeberg_muscle_explicit` is used when full torque control is needed without spring model interference.

---

## Torque Data Logging

After each `step()`, four values are written to `JointSensorArray` per joint:

```cython
self.joints_data.array[iteration, joint_data_i, JOINT_CMD_TORQUE]       = torque
self.joints_data.array[iteration, joint_data_i, JOINT_TORQUE_ACTIVE]    = active_torque
self.joints_data.array[iteration, joint_data_i, JOINT_TORQUE_STIFFNESS] = passive_stiffness
self.joints_data.array[iteration, joint_data_i, JOINT_TORQUE_DAMPING]   = damping
self.joints_data.array[iteration, joint_data_i, JOINT_TORQUE_FRICTION]  = friction
```

This allows post-simulation decomposition of which torque component dominated at any given time.

---

## Class Reference

### `EkebergMuscleCy`

**Source:** `farms_amphibious/control/ekeberg.pyx`
**Base:** `JointsMusclesCy`

| Attribute | Type | Description |
|---|---|---|
| `n_joints` | `int` | Number of joints with Ekeberg control |
| `parameters` | `float64[N_joints, 5]` | α, β, γ, δ, ε per joint |
| `osc_indices` | `uint32[2, N_joints]` | `[flexor_idx, extensor_idx]` per joint |
| `indices` | `uint32[N_joints]` | Joint sensor data indices |
| `joints_data` | `JointSensorArrayCy` | Shared sensor data array (read/write) |
| `state` | `OscillatorNetworkStateCy` | CPG state (read-only) |
| `transform_gain` | `float64[N_joints_total]` | Transform gain per joint (`gain` arg in constructor) |
| `transform_bias` | `float64[N_joints_total]` | Transform bias per joint (`bias` arg in constructor) |
| `activations` | `float64[N_osc]` | Current neural outputs (updated each step) |
| `joints_offsets` | `float64[N_joints]` | Current offset values (updated each step) |
| `spring_coefs` | `float64[N_joints]` | Live spring coefficients for MuJoCo |
| `damping_coefs` | `float64[N_joints]` | Live damping coefficients for MuJoCo |

| Method | Description |
|---|---|
| `step(iteration)` | Compute and log all torque components for current iteration |
| `update_activations(iteration)` | Refresh `activations` from `state.outputs(iteration)` |
| `update_offsets(iteration)` | Refresh `joints_offsets` from `state.offsets(iteration)` |
| `torques_implicit(iteration)` | Return `JOINT_TORQUE_ACTIVE` values (active torque only, after `step`) |

!!! warning "`springrefs` is not a method of `EkebergMuscleCy`"
    The `springrefs()`, `springcoefs()`, and `dampingcoefs()` methods live on the
    `JointMuscleController` (in `amphibious.py`), not on `EkebergMuscleCy` itself.
    `JointMuscleController.springrefs()` reads `EkebergMuscleCy.joints_offsets`
    (transformed during `step()`); `springcoefs()`/`dampingcoefs()` read
    `EkebergMuscleCy.spring_coefs`/`damping_coefs`. The full torque (logged to
    `JOINT_CMD_TORQUE`) is retrieved via the inherited `torque_cmds(iteration)`.

---

## See Also

- [CPG Oscillators](cpg-oscillators.md) — Source of all `outputs()` and `phases()` used here
- [Passive Joint Model](joint-controllers.md#1-passive-joint-passivejointcy) - For joints without CPG drive
- [Position Muscle](joint-controllers.md#2-position-muscle-positionmusclecy) - Alternative: position-mode control
