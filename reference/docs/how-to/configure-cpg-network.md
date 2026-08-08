# Configure CPG Network Parameters

This guide explains how to configure the CPG (Central Pattern Generator)
oscillator network used by `farms_amphibious` for locomotion control.

## Overview

The amphibious CPG network is defined in `AmphibiousNetworkOptions`
(`farms_amphibious/model/options.py`). It consists of:

- **Oscillators** — phase/amplitude oscillators that generate rhythmic patterns
- **Drives** — descending drive signals that modulate oscillator frequency and
  amplitude
- **Connections** — weighted connections between oscillators, sensors, and drives

## Enabling the CPG network

The CPG network is configured under `control.network` in `animat_config.yaml`:

```yaml
control:
  network:
    drives:
      - name: drv_body_L_0
        initial_value: 1.0
        kind: spine_left
        contacts: []
    oscillators:
      - name: osc_body_L_0
        initial_phase: 0.0
        initial_amplitude: 0.0
        frequency_gain: 1.0
        frequency_bias: 0.0
        # ... more fields
    osc2osc: [...]
    drive2osc: [...]
```

!!! note "When the network is used"
    The `AmphibiousControlOptions.__init__` only creates the network if the
    `network` key is present AND contains an `oscillators` sub-key. Otherwise
    `self.network` is set to `None`. When using a custom controller (like
    ZbotCPGController), the network section is typically not used — the
    controller manages its own oscillators internally.

## Network option keys

### Top-level network keys

| Key | Type | Required | Default | Parsed by | Notes |
|-----|------|----------|---------|-----------|-------|
| `drive_loader` | str | No | `''` | `AmphibiousNetworkOptions.__init__` | Dotted path to drive loader |
| `drive_config` | str | No | `''` | `AmphibiousNetworkOptions.__init__` | Drive configuration file |
| `drives` | list[dict] | Yes | `[]` | `AmphibiousDriveOptions` | Descending drives |
| `oscillators` | list[dict] | Yes | `[]` | `AmphibiousOscillatorOptions` | CPG oscillators |
| `single_osc_body` | bool | No | `False` | `AmphibiousNetworkOptions.__init__` | One oscillator per body joint |
| `single_osc_legs` | bool | No | `False` | `AmphibiousNetworkOptions.__init__` | One oscillator per leg joint |
| `osc2osc` | list[dict] \| null | No | `None` | `AmphibiousNetworkOptions.__init__` | Oscillator-to-oscillator connections |
| `joint2osc` | list[dict] \| null | No | `None` | `AmphibiousNetworkOptions.__init__` | Joint sensor → oscillator |
| `contact2osc` | list[dict] \| null | No | `None` | `AmphibiousNetworkOptions.__init__` | Contact sensor → oscillator |
| `xfrc2osc` | list[dict] \| null | No | `None` | `AmphibiousNetworkOptions.__init__` | External force → oscillator |
| `drive2osc` | list[int] \| null | No | `None` | `AmphibiousNetworkOptions.__init__` | Drive → oscillator mapping |
| `drive2joint` | list[list[int]] \| null | No | `None` | `AmphibiousNetworkOptions.__init__` | Drive → joint mapping |

### Oscillator options

Each entry in the `oscillators` list is parsed by `AmphibiousOscillatorOptions`:

| Key | Type | Required | Default | Notes |
|-----|------|----------|---------|-------|
| `name` | str | Yes | — | Oscillator name (e.g., `osc_body_L_0`) |
| `initial_phase` | float | Yes | — | Initial phase [rad] |
| `initial_amplitude` | float | Yes | — | Initial amplitude |
| `frequency_gain` | float | Yes | — | Frequency gain (multiplied by drive) |
| `frequency_bias` | float | Yes | — | Frequency bias [Hz] |
| `frequency_low` | float | Yes | — | Minimum frequency [Hz] |
| `frequency_high` | float | Yes | — | Maximum frequency [Hz] |
| `frequency_saturation_low` | float | Yes | — | Low saturation frequency |
| `frequency_saturation_high` | float | Yes | — | High saturation frequency |
| `amplitude_gain` | float | Yes | — | Amplitude gain (multiplied by drive) |
| `amplitude_bias` | float | Yes | — | Amplitude bias |
| `amplitude_low` | float | Yes | — | Minimum amplitude |
| `amplitude_high` | float | Yes | — | Maximum amplitude |
| `amplitude_saturation_low` | float | Yes | — | Low saturation amplitude |
| `amplitude_saturation_high` | float | Yes | — | High saturation amplitude |
| `rate` | float | Yes | — | Filter rate |
| `modular_phase` | float | No | `0` | Modular phase offset |
| `modular_amplitude` | float | No | `0` | Modular amplitude offset |

!!! todo "Saturation field inconsistency"
    The `defaults_from_convention()` method uses a `saturation` key (without
    `_low`/`_high` suffix) when generating default frequency/amplitude dicts,
    but `AmphibiousOscillatorOptions.__init__` expects `frequency_saturation_low`
    and `frequency_saturation_high`. This may indicate the defaults path and
    the explicit YAML path handle saturation differently. Verify against the
    actual `ode_oscillators_sparse` ODE function if you rely on saturation.

### Drive options

Each entry in the `drives` list is parsed by `AmphibiousDriveOptions`:

| Key | Type | Required | Default | Notes |
|-----|------|----------|---------|-------|
| `name` | str | Yes | — | Drive name (e.g., `drv_body_L_0`) |
| `initial_value` | float | Yes | — | Initial drive value |
| `kind` | str (DriveKind) | Yes | — | Drive type (see below) |
| `contacts` | list[tuple[str, str]] | Yes | — | Associated contact links |

### DriveKind enum

`DriveKind` (str Enum, in `farms_amphibious/model/options.py`):

| Value | String | Description |
|-------|--------|-------------|
| `BRAIN_LEFT` | `brain_left` | Left brain descending drive |
| `BRAIN_RIGHT` | `brain_right` | Right brain descending drive |
| `SPINE_LEFT` | `spine_left` | Left spinal drive |
| `SPINE_RIGHT` | `spine_right` | Right spinal drive |

### Connection types

Connections in `osc2osc`, `joint2osc`, `contact2osc`, and `xfrc2osc` are dicts
with these keys:

| Key | Type | Description |
|-----|------|-------------|
| `in` | str | Input oscillator name |
| `out` | str or tuple | Output target (oscillator name or sensor link name) |
| `type` | str | Connection type (see below) |
| `weight` | float | Connection weight |
| `phase_bias` | float | Phase bias [rad] (osc2osc only) |

Connection types used in the code:

| Type | Description |
|------|-------------|
| `OSC2OSC` | Oscillator to oscillator |
| `REACTION2FREQ` | Contact sensor to oscillator frequency |
| `LATERAL2FREQ` | External force (xfrc) to oscillator frequency |
| `LATERAL2AMP` | External force (xfrc) to oscillator amplitude |

## Automatic defaults via convention

When `AmphibiousOptions.from_options()` is used without explicit network
configuration, `defaults_from_convention()` auto-generates:

- Oscillator names and initial phases based on body/leg convention
- Default frequencies (gain=0, bias=0, low=1, high=5 for body; low=1, high=3
  for legs)
- Default amplitudes (all zeros unless `body_walk_amplitude` etc. specified)
- Default rates (10.0 for all oscillators)
- Default connectivity (standing wave with phase lag along body)

The convention is determined by `AmphibiousConvention`, which uses morphology
parameters (`n_joints_body`, `n_legs`, `n_dof_legs`) to compute oscillator
counts, naming, and connectivity.

## The ODE integrator

The CPG network is integrated by `NetworkODE` (`farms_amphibious/control/network.py`):

```python
class NetworkODE(AnimatNetwork):
    def __init__(self, data, integrator='dopri5', **kwargs):
        self.ode = kwargs.pop('ode', ode_oscillators_sparse)
        self.solver = integrate.ode(f=self.ode)
        self.solver.set_integrator(self.integrator, **self.integrator_kwargs)
        self.solver.set_initial_value(y=data.state.array[0, :], t=0.0)
```

- Uses `scipy.integrate.ode` with the `dopri5` (Dormand-Prince) integrator by
  default
- The ODE function `ode_oscillators_sparse` computes phase and amplitude
  derivatives from drive inputs, oscillator parameters, and connectivity
- A `modulo` parameter controls how often the ODE is integrated (default: 1,
  meaning every step)

## Muscle mapping

Oscillator outputs are mapped to joints via `AmphibiousMuscleSetOptions`:

| Key | Type | Description |
|-----|------|-------------|
| `joint_name` | str | Target joint |
| `osc1` | str | Primary oscillator name |
| `osc2` | str | Antagonist oscillator name |
| `alpha` | float | Gain |
| `beta` | float | Stiffness gain |
| `gamma` | float | Tonic gain |
| `delta` | float | Damping coefficient |
| `epsilon` | float | Friction coefficient |

## See also

- [CPG Control Architecture](../explanation/cpg-architecture.md) — design rationale
- [Configure an Experiment YAML](configure-yaml.md) — overall YAML structure
- [farms_amphibious Reference](../reference/amphibious/farms-amphibious.md) — full API
