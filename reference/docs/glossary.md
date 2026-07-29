# Glossary — Domain terminology and implementations

This glossary defines core domain terminology used across the FARMS framework and cross-references where these concepts are actually implemented in the codebase.

!!! note "Source Files"
    - `farms_amphibious/farms_amphibious/control/ode.pyx` — CPG oscillator implementation
    - `farms_amphibious/farms_amphibious/control/network.py` — `NetworkODE` integration
    - `farms_mujoco/farms_mujoco/simulation/task.py` — `ExperimentTask` lifecycle
    - `farms_mujoco/farms_mujoco/simulation/simulation.py` — `Simulation` backend

## A
**Added Mass**  
The inertia added to a system because an accelerating or decelerating body must move some volume of surrounding fluid as it moves through it.  
*Implementation Status*: Despite mentions in older documentation, added mass is **not currently implemented** in the active physics backend. See `farms_mujoco/swimming/drag.pyx`.

**Animat**  
A portmanteau of "animal" and "robot". It represents the primary simulated entity in FARMS.  
*Implementation*: 
- Configuration: `farms_core.model.options.AnimatOptions`
- Memory structures: `farms_core.model.data.AnimatData`
- Controller Interface: `farms_core.model.control.AnimatController`

## C
**CPG (Central Pattern Generator)**  
A biological neural network capable of producing coordinated rhythmic output signals (like walking or swimming) without any rhythmic inputs. Modeled in FARMS as a system of coupled oscillators.  
*Implementation*: 
- Math: `farms_amphibious/control/ode.pyx`
- Integration Loop: `farms_amphibious.control.network.NetworkODE`
- Controller wrapper: `farms_amphibious.control.amphibious.AmphibiousController`

## D
**dm_control**  
Google DeepMind's software stack for physics-based simulation and Reinforcement Learning environments, utilizing MuJoCo. FARMS heavily leverages this instead of raw MuJoCo bindings.  
*Implementation*: The entire `farms_mujoco` backend is built around `dm_control.rl.control.Task` and `dm_control.rl.control.Environment`. See `farms_mujoco/simulation/task.py` and `farms_mujoco/simulation/simulation.py`.

## E
**Ekeberg Muscle Model**  
A phenomenological muscle model that translates neural excitation into joint torque, incorporating active force, passive stiffness, active stiffness, and damping.  
*Implementation*: `farms_amphibious/control/ekeberg.pyx`

## M
**MJCF (MuJoCo XML Format)**  
An XML format designed explicitly for the MuJoCo physics engine to define rigid body dynamics, joints, and actuators. FARMS dynamically generates MJCF files from user configurations.  
*Implementation*: `farms_mujoco/simulation/mjcf.py`

## S
**SDF (Simulation Description Format)**  
An XML format originally developed for the Gazebo simulator, used to describe objects and environments. FARMS uses SDF to define animat morphologies.  
*Implementation*: Parsed via `farms_core.io.sdf`.

## T
**TaskExtension**  
The core plugin architecture of the FARMS simulation loop. Any custom logic (controllers, hydrodynamic forces, loggers) that needs to run per-timestep must inherit from this class.  
*Implementation*: `farms_core.simulation.extensions.TaskExtension`

**Tegotae**  
A Japanese concept translating roughly to "response" or "reaction". In FARMS CPG models, it refers to a specific form of sensory feedback where the CPG phase is modulated by the interaction of joint stretch and the current oscillator phase ($\theta_{joint} \cdot \sin(\theta_{cpg})$).  
*Implementation*: `farms_amphibious/control/ode.pyx`
