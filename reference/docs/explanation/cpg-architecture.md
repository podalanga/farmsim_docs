# CPG Control Architecture

This document explains the design of FARMS' Central Pattern Generator (CPG)
locomotion control system, as implemented in `farms_amphibious`.

## What is a CPG?

A Central Pattern Generator is a neural network model that produces rhythmic
motor patterns without requiring rhythmic sensory input. In biology, CPGs in
the spinal cord generate walking, swimming, and flying gaits. In FARMS, the
CPG is modeled as a network of coupled phase/amplitude oscillators.

## Oscillator model

Each oscillator in the FARMS CPG has:

- **Phase** (\(\phi\)) — angular position in the oscillation cycle
- **Amplitude** (\(R\)) — magnitude of the oscillation
- **Frequency** — determined by drive input × frequency_gain + frequency_bias
- **Post-synaptic targets** — connected joints and muscles

The oscillator dynamics are integrated by `ode_oscillators_sparse` (a Cython
function in `farms_amphibious/control/ode.py`), using scipy's ODE solver
(`dopri5` by default).

## Network components

```
Drives (descending signals)
    ↓
Oscillators (phase/amplitude)
    ↓
Muscle mapping (osc → joint)
    ↓
Joint equations (phase, Ekeberg, passive)
    ↓
Joint targets (position/velocity/torque)
```

### Drives

Drives (`AmphibiousDriveOptions`) are scalar signals that modulate oscillator
frequency and amplitude. Each drive has:

- `initial_value` — starting drive strength
- `kind` — `BRAIN_LEFT`, `BRAIN_RIGHT`, `SPINE_LEFT`, `SPINE_RIGHT`
- `contacts` — associated contact sensor links

Drives represent descending commands from higher neural centers (brain) or
spinal pattern generators.

### Oscillators

Each oscillator (`AmphibiousOscillatorOptions`) has:

- Frequency: `gain × drive + bias`, clamped to `[low, high]` range
- Amplitude: `gain × drive + bias`, clamped to `[low, high]` range
- Initial phase and amplitude
- Filter rate for smooth transitions

The frequency and amplitude are both modulated by the drive signal, allowing
a single scalar (drive) to control both the speed and strength of oscillation.

### Connectivity

The network supports five connection types:

| Connection | From | To | Effect |
|------------|------|----|--------|
| `osc2osc` | Oscillator | Oscillator | Phase coupling (with phase bias) |
| `joint2osc` | Joint sensor | Oscillator | Stretch reflex (frequency/amplitude modulation) |
| `contact2osc` | Contact sensor | Oscillator | Ground contact feedback |
| `xfrc2osc` | External force | Oscillator | Fluid force feedback |
| `drive2osc` | Drive | Oscillator | Drive → oscillator mapping |

Each connection has a `weight` that determines the strength of coupling.
Phase coupling connections also have a `phase_bias` that sets the desired
phase difference between oscillators.

### Muscle mapping

`AmphibiousMuscleSetOptions` maps oscillator outputs to joints:

- Each joint is driven by two oscillators (agonist `osc1` and antagonist `osc2`)
- Five parameters (\(\alpha, \beta, \gamma, \delta, \epsilon\)) control the
  muscle model: gain, stiffness, tonic, damping, and friction

### Joint equations

The controller supports multiple equations for converting oscillator outputs
to joint commands:

| Equation | Description | Control types |
|----------|-------------|---------------|
| `phase` | Direct phase-to-position mapping | position |
| `position_muscle` | Position with muscle activation | position |
| `ekeberg_muscle` | Ekeberg neuron + muscle model | velocity, torque |
| `ekeberg_muscle_explicit` | Explicit Ekeberg torque | torque |
| `passive` | Spring-damper (no active control) | velocity, torque |
| `passive_explicit` | Explicit passive torque | torque |

This allows different joints to use different control strategies — for example,
body joints might use `phase` while leg joints use `ekeberg_muscle`.

## Convention-based generation

`AmphibiousConvention` automatically generates:

- Oscillator names and indices based on body/leg structure
- Default connectivity (standing wave with phase lag along body)
- Default frequencies and amplitudes
- Drive names and kinds

This allows minimal YAML configuration — specify only the morphology
(`n_joints_body`, `n_legs`, `n_dof_legs`) and the convention generates the full
network.

### Standing wave pattern

The default connectivity creates a standing wave along the body: neighboring
body segments oscillate with a phase lag of \(2\pi / n_{joints\_body}\),
producing a traveling wave from head to tail. Left and right side oscillators
are anti-phase (\(\pi\) phase bias), producing lateral undulation.

## ODE integration

`NetworkODE` (`farms_amphibious/control/network.py`) integrates the CPG:

- Uses `scipy.integrate.ode` with `dopri5` (Dormand-Prince) integrator
- The `modulo` parameter allows integrating less frequently than the physics
  step (e.g., `modulo=5` integrates the CPG every 5 physics steps)
- On integration failure: warns and resets to previous state (or raises if
  `strict=True`)

The ODE function `ode_oscillators_sparse` computes:

- Phase derivatives: \(\dot{\phi}_i = \omega_i + \sum_j w_{ij} \sin(\phi_j - \phi_i - \theta_{ij})\)
- Amplitude derivatives: first-order filter toward drive-dependent target

where \(\omega_i\) is the drive-dependent frequency, \(w_{ij}\) are connection
weights, and \(\theta_{ij}\) are phase biases.

## Custom controllers

When using a custom controller (like `ZbotCPGController`), the built-in CPG
network is typically bypassed. The custom controller implements its own
oscillator model (e.g., `SegmentalCPG`) and is registered as an animat
extension. The `control.network` section in YAML is left commented out.

This design allows the framework's CPG infrastructure to be used when
appropriate, while also supporting fully custom locomotion controllers that
follow the same `AnimatController` interface.

## See also

- [Configure CPG Network Parameters](../how-to/configure-cpg-network.md) —
  YAML configuration
- [Write a Custom Controller](../tutorials/custom-controller.md) — custom
  controller tutorial
- [farms_amphibious Reference](../reference/amphibious/farms-amphibious.md) — API reference
