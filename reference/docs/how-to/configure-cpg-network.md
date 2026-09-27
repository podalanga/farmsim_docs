# Configure CPG Network Parameters

How to configure the CPG (Central Pattern Generator) network that
`AmphibiousController` (`farms_amphibious`) uses to drive the joints. The
equations are in [Mathematical Models](../explanation/mathematical-models.md),
the design in [CPG Control Architecture](../explanation/cpg-architecture.md),
and the Zbot values in [Swimming Experiment](../tutorials/zbot-experiment.md#cpg-network).

## Where it is configured

The network is `control.network` of `animat_config.yaml`
(`AmphibiousNetworkOptions`), and the joints use it through
`control.muscles` and the motor `equation`:

```yaml
control:
  motors:
    - joint_name: joint_1
      control_types: [position]
      equation: position_muscle
      # ...
  network:
    drive_loader: ''
    drive_config: ''
    drives: [...]
    oscillators: [...]
    osc2osc: [...]
    drive2osc: [...]
    drive2joint: [...]
    joint2osc: []
    contact2osc: []
    xfrc2osc: []
  muscles: [...]
extensions:
  - loader: farms_amphibious.control.amphibious.AmphibiousController
    config: {}
```

The network is only created when `control.network` has an `oscillators`
key, and it is only used when `AmphibiousController` is in the animat's
`extensions:`. A custom controller (such as `ZbotCPGController`) ignores
it.

## Top-level keys

| Key | Required | Description |
|-----|----------|-------------|
| `drive_loader` | No (`''`) | Dotted path of a descending drive class (see [Descending Drive](../reference/amphibious/descending-drive.md)). Empty: the drives keep their initial values |
| `drive_config` | Yes | Configuration of the drive loader (`''` when unused) |
| `drives` | Yes | Descending drives |
| `oscillators` | Yes | Oscillators |
| `single_osc_body`, `single_osc_legs` | No (`false`) | One oscillator per joint instead of two |
| `osc2osc` | No | Couplings between oscillators |
| `drive2osc` | No | Drive of each oscillator |
| `drive2joint` | No | Drives setting the offset of each joint |
| `joint2osc`, `contact2osc`, `xfrc2osc` | No | Sensory feedback |

## Drives

```yaml
drives:
  - name: drive_body_0_L
    initial_value: 4
    kind: null          # brain_left, brain_right, spine_left, spine_right or null
    contacts: []
```

`kind` (`DriveKind`) is used by descending drive controllers such as
`OrientationFollower` to know which drives to change.

## Oscillators

```yaml
oscillators:
  - name: osc_body_0_L
    initial_phase: 1.0489             # [rad]
    initial_amplitude: 0.0
    frequency_gain: 1.5708            # [rad/s per drive unit]
    frequency_bias: 0.0               # [rad/s]
    frequency_low: 1                  # Drive range where the linear rule applies
    frequency_high: 5
    frequency_saturation_low: 0       # [rad/s] below frequency_low
    frequency_saturation_high: 0      # [rad/s] above frequency_high
    amplitude_gain: 0.15
    amplitude_bias: 0.0
    amplitude_low: 0.9
    amplitude_high: 5
    amplitude_saturation_low: 0
    amplitude_saturation_high: 0.75
    rate: 3.0                         # Amplitude convergence rate [1/s]
    modular_phase: 0                  # Optional frequency modulation
    modular_amplitude: 0
```

The intrinsic angular frequency and the nominal amplitude follow the
same rule from the drive $d$ of the oscillator:

- `gain*d + bias` when `low <= d <= high`;
- `saturation_low` when `d < low`, `saturation_high` when `d > high`.

The saturation values are not clamps: with `frequency_saturation_high: 0`,
a drive above `frequency_high` stops the oscillator. With `modular_amplitude`
above 0.001, the frequency is multiplied by
`1 + modular_amplitude*cos(phase + modular_phase)`.

## Connections

### osc2osc

```yaml
osc2osc:
  - in: osc_body_1_L      # The oscillator receiving the coupling
    out: osc_body_0_L     # The oscillator it is coupled to
    type: OSC2OSC
    weight: 30.0
    phase_bias: 1.0472    # At steady state, phase(out) - phase(in) = phase_bias
```

This entry makes `osc_body_0_L` lead `osc_body_1_L` by 60°. A coupling is
one-way: add the reverse entry (with `phase_bias` $-\varphi$, or
$2\pi - \varphi$) for a symmetric coupling.

### drive2osc and drive2joint

```yaml
drive2osc:
  - drive: drive_body_0_L
    oscillator: osc_body_0_L
drive2joint:
  - drive0: drive_body_0_L
    drive1: drive_body_0_R
    joint: joint_1
```

With `drive2joint`, the joint offset converges (at the motor's
`offsets.rate`) to `offsets.gain*(drive1 - drive0) + offsets.bias`, while
the mean of the two drives is within `[offsets.low, offsets.high]`: a
difference between the left and right drives bends the body, to turn.

### Sensory feedback

`joint2osc`, `contact2osc` and `xfrc2osc` entries have `in` (the
oscillator), `out` (the joint or link), `type` and `weight`. The types
(`ConnectionType`, `farms_amphibious/data/data_cy.pxd`):

| Connection | Types | Effect |
|------------|-------|--------|
| `joint2osc` | `STRETCH2FREQ`, `STRETCH2AMP`, `STRETCH2FREQTEGOTAE`, `STRETCH2AMPTEGOTAE` | Joint position times the weight (times `sin(phase)` for Tegotae) added to the phase or amplitude derivative |
| `contact2osc` | `REACTION2FREQ`, `REACTION2FREQTEGOTAE` | Norm of the total contact force |
| `xfrc2osc` | `LATERAL2FREQ`, `LATERAL2AMP` | Absolute y component (world frame) of the external force of the link |

## Muscles

`control.muscles` maps each joint to its two oscillators:

```yaml
muscles:
  - joint_name: joint_1
    osc1: osc_body_0_L
    osc2: osc_body_0_R
    alpha: 0.5     # Active gain
    beta: 1.0      # Stiffness
    gamma: 0.1     # Passive stiffness ratio
    delta: 0.001   # Damping
    epsilon: 0     # Friction
```

`position_muscle` and `position_phase` use `osc1` and `osc2`; the
coefficients are only used by the `ekeberg_muscle` equations.

## Defaults from the convention

`AmphibiousOptions.from_options()` (options built in Python) can generate
the oscillators, drives and couplings from the morphology
(`n_joints_body`, `n_legs`, `n_dof_legs`) with `AmphibiousConvention`.

!!! warning "Known issue"
    When no oscillator is given, `AmphibiousNetworkOptions.defaults_from_convention()`
    creates them with `frequency_saturation` and `amplitude_saturation`
    keys, while the oscillator options use `*_saturation_low` and
    `*_saturation_high`. Write the oscillators explicitly, as the Zbot
    configuration does.

## Integration

`NetworkODE` (`farms_amphibious/control/network.py`) integrates the network
with SciPy's `dopri5` at every environment step. Its `modulo` option
integrates only every `modulo` iterations. See
[ODE Internals](../internals/ode-internals.md).

## See also

- [CPG Control Architecture](../explanation/cpg-architecture.md)
- [Configure an Experiment YAML](configure-yaml.md)
- [farms_amphibious Reference](../reference/amphibious/farms-amphibious.md)
