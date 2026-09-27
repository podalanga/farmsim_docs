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

- a **phase** \(\theta\), its position in the oscillation cycle;
- an **amplitude** \(A\), which converges to a nominal amplitude;
- an **output** \(A (1 + \cos\theta)\), read by the joint equations.

Its intrinsic frequency and nominal amplitude depend on its drive. The
dynamics are computed by `ode_oscillators_sparse` (Cython,
`farms_amphibious/control/ode.pyx`) and integrated with SciPy's `dopri5`.

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

- `initial_value`: starting drive value
- `kind`: `brain_left`, `brain_right`, `spine_left`, `spine_right` or
  `null` (`DriveKind`), used by the descending drive controllers
- `contacts`: associated contact sensor links

Without a `drive_loader`, the drives keep their initial values.

Drives represent descending commands from higher neural centers (brain) or
spinal pattern generators.

### Oscillators

Each oscillator (`AmphibiousOscillatorOptions`) has:

- Frequency [rad/s]: `frequency_gain × drive + frequency_bias` while the
  drive is in `[frequency_low, frequency_high]`, else
  `frequency_saturation_low` or `frequency_saturation_high`
- Nominal amplitude: the same rule with the `amplitude_*` parameters
- Initial phase and amplitude
- `rate`: convergence rate of the amplitude

The frequency and amplitude are both modulated by the drive signal, allowing
a single scalar (drive) to control both the speed and strength of oscillation.

### Connectivity

The network has these connection types:

| Connection | From | To | Effect |
|------------|------|----|--------|
| `osc2osc` | Oscillator | Oscillator | Phase coupling (with phase bias) |
| `joint2osc` | Joint sensor | Oscillator | Stretch reflex (frequency/amplitude modulation) |
| `contact2osc` | Contact sensor | Oscillator | Ground contact feedback |
| `xfrc2osc` | External force | Oscillator | Fluid force feedback |
| `drive2osc` | Drive | Oscillator | Drive of each oscillator |
| `drive2joint` | Two drives | Joint | Joint offset from the drive difference |

Each connection has a `weight` that determines the strength of coupling.
Phase coupling connections also have a `phase_bias` that sets the desired
phase difference between oscillators.

### Muscle mapping

`AmphibiousMuscleSetOptions` maps oscillator outputs to joints:

- Each joint is driven by two oscillators (`osc1` and `osc2`, usually left
  and right)
- The Ekeberg equations use five parameters: \(\alpha\) (active gain),
  \(\beta\) (stiffness), \(\gamma\) (passive stiffness ratio),
  \(\delta\) (damping) and \(\epsilon\) (friction). See
  [Mathematical Models](mathematical-models.md#joint-equations).

### Joint equations

The controller supports multiple equations for converting oscillator outputs
to joint commands:

| Equation | Description | Control types |
|----------|-------------|---------------|
| `position_phase` | Position from the oscillator phase | position |
| `position_muscle` | Position from the difference of the two outputs (used by the Zbot) | position |
| `ekeberg_muscle` | Ekeberg muscle model, stiffness and damping through the MuJoCo joint | velocity, torque |
| `ekeberg_muscle_explicit` | Ekeberg muscle model, all terms as a torque | torque |
| `passive` | Passive spring-damper, no active control | velocity, torque |

This allows different joints to use different control strategies, for
example position control for the body and Ekeberg muscles for the legs.

## Convention-based generation

`AmphibiousConvention` automatically generates:

- Oscillator names and indices based on body/leg structure
- Default connectivity (standing wave with phase lag along body)
- Default frequencies and amplitudes
- Drive names and kinds

This allows minimal YAML configuration: specify only the morphology
(`n_joints_body`, `n_legs`, `n_dof_legs`) and the convention generates the full
network.

### Travelling wave pattern

The default connectivity creates a travelling wave along the body: neighboring
body segments oscillate with a phase lag of \(2\pi / n_{joints\_body}\),
producing a traveling wave from head to tail. Left and right side oscillators
are anti-phase (\(\pi\) phase bias), producing lateral undulation.

## ODE integration

`NetworkODE` (`farms_amphibious/control/network.py`) integrates the CPG:

- Uses `scipy.integrate.ode` with the `dopri5` (Dormand-Prince) integrator
- The `modulo` parameter integrates less often: with `modulo=5`, the state
  is only integrated every 5 iterations and copied in between
- On integration failure: warns and restarts from the current solver
  state, or raises with `strict=True`

The ODE function `ode_oscillators_sparse` computes:

- Phase derivatives: \(\dot{\theta}_i = \omega_i + \sum_j A_j w_{ij} \sin(\theta_j - \theta_i - \varphi_{ij})\), plus the sensory feedback terms
- Amplitude derivatives: \(\dot A_i = a_i (R_i - A_i)\), a first-order filter toward the drive-dependent nominal amplitude \(R_i\)
- Joint offset derivatives, toward the offset set by the drives

where \(\omega_i\) is the drive-dependent frequency, \(w_{ij}\) are the
connection weights and \(\varphi_{ij}\) the phase biases (`in` is \(i\),
`out` is \(j\) in `osc2osc`).

## Custom controllers

When using a custom controller (like `ZbotCPGController`), the built-in CPG
network is typically bypassed. The custom controller implements its own
oscillator model (e.g., `SegmentalCPG`) and is registered as an animat
extension, in place of `AmphibiousController`. In `zbot_bout_glide`, the
`control.network` section is commented out.

This design allows the framework's CPG infrastructure to be used when
appropriate, while also supporting fully custom locomotion controllers that
follow the same `AnimatController` interface.

## See also

- [Configure CPG Network Parameters](../how-to/configure-cpg-network.md): 
  YAML configuration
- [Write a Custom Controller](../tutorials/custom-controller.md): custom
  controller tutorial
- [farms_amphibious Reference](../reference/amphibious/farms-amphibious.md): API reference
