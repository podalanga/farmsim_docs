# farms_amphibious.control.network

Numerical ODE integrator for CPG oscillator networks.

## Overview

The `farms_amphibious.control.network` module handles the numerical integration of Central Pattern Generator (CPG) networks alongside the MuJoCo physics engine. It provides the abstractions and concrete implementations necessary to step the coupled nonlinear oscillators forward in time, ensuring stable limits cycles and consistent gait generation.

---

## AnimatNetwork

Abstract base class representing any neural control network.

```python
def __init__(self, data, n_iterations)
```

| Name | Type | Default | Description |
| --- | --- | --- | --- |
| `data` | `AnimatData` | _Required_ | Reference to the complete simulation data structure. |
| `n_iterations` | `int` | _Required_ | Total number of simulation iterations. |

### `step`

```python
@abstractmethod
def step(self, iteration: int, time: float, timestep: float, **kwargs)
```

Abstract method called at each simulation iteration to advance the network state.

| Name | Type | Default | Description |
| --- | --- | --- | --- |
| `iteration` | `int` | _Required_ | Current simulation iteration index. |
| `time` | `float` | _Required_ | Current simulation time in seconds. |
| `timestep` | `float` | _Required_ | Simulation timestep in seconds. |

---

## NetworkODE

Concrete implementation of `AnimatNetwork` using SciPy's ODE solvers for CPG integration.

```python
def __init__(self, data, integrator='dopri5', **kwargs)
```

| Name | Type | Default | Description |
| --- | --- | --- | --- |
| `data` | `AnimatData` | _Required_ | Reference to the complete simulation data structure. |
| `integrator` | `str` | `'dopri5'` | SciPy ODE integrator to use. |
| `modulo` | `int` | `1` | Number of physics steps per ODE integration step. |
| `ode` | `Callable` | `ode_oscillators_sparse` | Derivative function for the CPG network. |

### `initialize_episode`

```python
def initialize_episode(self)
```

Resets the solver, re-initializes the ODE integrator, and clears the state arrays for a new episode.

### `copy_next_drive`

```python
def copy_next_drive(self, iteration)
```

Copies the drive array values from the current iteration to the next. Drive values persist indefinitely until explicitly changed by the descending drive system.

| Name | Type | Default | Description |
| --- | --- | --- | --- |
| `iteration` | `int` | _Required_ | Current simulation iteration index. |

### `step`

```python
def step(self, iteration: int, time: float, timestep: float, checks: bool = False, strict: bool = False)
```

Full orchestration of the neural integration step. Handles multi-rate execution, robust error recovery, and forward drive propagation.

- **Iteration 0**: Only calls `copy_next_drive(iteration)`.
- **Modulo skip**: If `iteration % modulo != 0`, copies the previous state forward without integrating.
- **Solver**: Sets the derivative parameters and integrates while `solver.t < time + 0.99*timestep`.
- **Error handling**: If integration fails and `strict=True`, raises an `IntegrationException`. Otherwise, attempts to restart the solver from the previous state.

| Name | Type | Default | Description |
| --- | --- | --- | --- |
| `iteration` | `int` | _Required_ | Current simulation iteration index. |
| `time` | `float` | _Required_ | Current simulation time in seconds. |
| `timestep` | `float` | _Required_ | Simulation timestep in seconds. |
| `checks` | `bool` | `False` | Whether to perform strict validation assertions during the step. |
| `strict` | `bool` | `False` | Whether to raise an exception on integration failure. |

---

## Integration Strategy

### Why `dopri5`?

Coupled oscillators can become stiff equations, especially when subjected to discontinuous sensory feedback such as foot strikes. Simple fixed-step methods (like Euler integration) cause rapid phase drift or catastrophic instability in CPGs. The default `'dopri5'` (Dormand-Prince Runge-Kutta 4(5)) method provides adaptive step sizing with high-order accuracy, ensuring the phases of the oscillators remain mathematically pure regardless of external perturbations.

### Multi-Rate Integration

Physics engines often require extremely small timesteps (e.g., $1 \text{ms}$) to resolve collisions stably. However, neural CPG networks evolve comparatively slowly.

The `modulo` parameter enables multi-rate integration. If `modulo = 5`, the computationally expensive ODE solver only steps once every 5 physics iterations. During skipped iterations, `NetworkODE` simply repeats the previous neural state, vastly accelerating the simulation without compromising physics stability.

---

## Usage Example

Stepping `NetworkODE` inside a `before_step` callback:

```python
def before_step(self, iteration, time, timestep):
    # Execute neural integration before physics
    self.network.step(
        iteration=iteration,
        time=time,
        timestep=timestep,
        strict=False
    )
```

## See Also

- [CPG Oscillators](cpg_oscillators.md) — Mathematical foundation for CPG dynamics
- [Amphibious Controller](farms_amphibious_controller.md) — How the network is wired into the control loop

Source: `farms_amphibious/control/network.py`
