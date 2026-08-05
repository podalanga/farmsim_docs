
# Architecture & Data Flow

This document details the cross-module dependencies and execution loop of the FARMS framework.

## Cross-Module Data Flow

The following data flow diagram illustrates how `farms_core` data structures flow into `farms_sim`, are consumed by the execution loop in `farms_mujoco`, and how `farms_amphibious` and hydrodynamics hook into this loop.

```mermaid
flowchart TD
    subgraph farms_core ["farms_core (Data Structures & Interfaces)"]
        EO[ExperimentOptions] -->|contains| AO[AnimatOptions]
        EO -->|contains| ArO[ArenaOptions]
        ED[ExperimentData] -->|contains| AD[AnimatData]
    end

    subgraph farms_sim ["farms_sim (Simulation Manager)"]
        FS_Sim["simulation.py: setup_from_clargs / run_simulation"]
        FS_Sim -->|loads| EO
        FS_Sim -->|initializes| ED
    end

    subgraph farms_mujoco ["farms_mujoco (Physics Backend)"]
        FM_Sim[Simulation.from_experiment]
        Task[ExperimentTask]
        Physics["dm_control Physics"]
        Env["dm_control Environment"]
        FM_Sim -->|creates| Task
        FM_Sim -->|creates| Physics
        FM_Sim -->|creates| Env
        Task -->|"reads/writes state"| ED
    end

    subgraph farms_amphibious ["farms_amphibious (Domain Control)"]
        AC["AmphibiousController / ZbotCPGController"]
        AC -.->|implements| TaskExtension[TaskExtension]
        AC -->|"computes torques/positions"| Task
    end

    subgraph Hydrodynamics ["farms_mujoco.swimming (Domain Physics)"]
        SE[SwimmingExtension]
        SH[SwimmingHandler]
        SE -.->|implements| TaskExtension
        SE -->|uses| SH
        SH -->|"applies xfrc_applied"| Physics
    end

    FS_Sim -->|creates| FM_Sim
    Task -->|"calls before_step()"| AC
    Task -->|"calls before_step()"| SE
```

## Class & Inheritance Diagrams

### `farms_core`

```mermaid
classDiagram
    class Options {
        <<dict subclass>>
        +from_options(cls, options)
        +load(cls, filename)
        +save(cls, filename)
    }
    class ExperimentOptions
    class AnimatOptions
    class ArenaOptions
    Options <|-- ExperimentOptions
    Options <|-- AnimatOptions
    Options <|-- ArenaOptions

    class TaskExtension {
        <<ABC>>
        +initialize_episode()
        +before_step()
        +after_step()
    }
    class AnimatExtension {
        <<ABC>>
        +animat_i
    }
    TaskExtension <|-- AnimatExtension
    class AnimatController {
        <<ABC>>
        +positions()
        +velocities()
        +torques()
        +springrefs()
        +springcoefs()
        +dampingcoefs()
        +excitations()
    }
    AnimatExtension <|-- AnimatController
```

### `farms_sim`

```mermaid
classDiagram
    class SimulationManager {
        <<Namespace>>
        +setup_from_clargs()
        +simulation_setup()
        +run_simulation()
    }
    note for SimulationManager "farms_sim acts as a functional entrypoint\nand simulation runner"
```

### `farms_mujoco`

```mermaid
classDiagram
    class dm_control_Task {
        <<dm_control>>
    }
    class ExperimentTask {
        +extensions : List~TaskExtension~
        +initialize_episode()
        +before_step()
        +after_step()
    }
    dm_control_Task <|-- ExperimentTask
    class Simulation {
        +physics : mjcf.Physics
        +task : ExperimentTask
        +run()
    }
    Simulation *-- ExperimentTask
    class SwimmingExtension
    class SwimmingHandler
    SwimmingExtension *-- SwimmingHandler
```

### `farms_amphibious`

```mermaid
classDiagram
    class AnimatController {
        <<farms_core>>
    }
    class JointMuscleController
    class AmphibiousController {
        +network : NetworkODE
    }
    class KinematicsController
    AnimatController <|-- JointMuscleController
    JointMuscleController <|-- AmphibiousController
    AnimatController <|-- KinematicsController
    class AnimatNetwork {
        <<Base>>
    }
    class NetworkODE {
        +step()
    }
    AnimatNetwork <|-- NetworkODE
    AmphibiousController *-- NetworkODE
```

## Execution Loop Reconstruction

The FARMS execution loop involves coordination between `farms_sim` (orchestration), `farms_mujoco` (task definitions), `dm_control` (RL environment loop), and `farms_amphibious` (forces/control).

```mermaid
sequenceDiagram
    participant App as farms_sim (simulation.py)
    participant Sim as farms_mujoco.Simulation
    participant Env as dm_control Environment
    participant Task as ExperimentTask
    participant Phys as MuJoCo Physics
    participant CPG as Controller (TaskExtension)
    participant Swim as SwimmingExtension (TaskExtension)

    App->>Sim: run_simulation()
    Sim->>Sim: Simulation.from_experiment()
    Note over Sim: setup_mjcf_xml() builds MJCF from SDF + arena
    Sim->>Task: ExperimentTask(...)
    Sim->>Phys: Physics.from_mjcf_model(mjcf)
    Sim->>Env: Environment(physics, task)

    loop Every Iteration
        loop cb_sub_steps times
            Sim->>Env: env.step(action=None)
            Env->>Task: task.before_step(physics)
            Task->>Task: update_sensors(physics)
            Note over Task: Copies physics.data (qpos, xpos) into AnimatData.sensors
            Task->>CPG: extension.before_step()
            CPG->>CPG: network.step() or controller.step()
            CPG-->>Task: returns torques/positions
            Task->>Phys: writes to physics.data.ctrl
            Task->>Swim: extension.before_step()
            Swim->>Swim: handler.step()
            Swim-->>Phys: writes to physics.data.xfrc_applied
            Note over Task,Swim: Order depends on config YAML array order!
            Env->>Phys: physics.step()
            Note over Phys: MuJoCo internal physics integration step
            Env->>Task: task.after_step(physics)
            Task->>CPG: extension.after_step()
            Task->>Swim: extension.after_step()
        end
    end
```

## Potential Fragility in Order-of-Operations

1. **Extension Ordering**: `Task.extensions` is populated by concatenating `simulation.extensions` and `animat.extensions` arrays from the user's YAML config (`ExperimentTask.extract_extensions`). `ExperimentTask` iterates this array sequentially. If the `SwimmingExtension` (hydrodynamics) is listed before the controller, the hydrodynamics model computes forces based on the *previous* step's positions/velocities before the controller actuates the joints.

2. **Buffer Overwrites**: `update_sensors()` writes into `AnimatData.sensors` using modulo arithmetic `index = task.iteration % buffer_size`. If an extension processes history longer than `buffer_size`, it will encounter overwritten data.