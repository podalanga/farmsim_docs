# Glossary

This glossary defines the terms used in FARMS and points to where each one is implemented in the code.

## A

**Added Mass**
The inertia added to a system because an accelerating or decelerating body must move some volume of surrounding fluid as it moves through it.
*Implementation*: with `fluid_model: ellipsoid`, Lamb's added mass of the ellipsoid fitted to each link (`added_mass: implicit` or `explicit`). See `farms_mujoco/swimming/ellipsoid_model.pyx` and [farms_mujoco.swimming](../reference/mujoco/mujoco-swimming.md).

**Centre of Buoyancy (CoB)**
The centroid of the submerged volume of a body, where buoyancy is applied. Computed per link from its geoms (`cob_method`).
*Implementation*: `farms_mujoco/swimming/cob.pyx` (exact), `farms_mujoco/swimming/cob_lut.pyx` (lookup tables)

**Animat**
A portmanteau of "animal" and "robot". It represents the primary simulated entity in FARMS.
*Implementation*:
- Configuration: `farms_core.model.options.AnimatOptions`
- Memory structures: `farms_core.model.data.AnimatData`
- Controller Interface: `farms_core.model.control.AnimatController`

**AmphibiousController**
The primary CPG-based controller for amphibious robots. Extends `JointMuscleController` and integrates a `NetworkODE` for oscillator dynamics.
*Implementation*: `farms_amphibious.control.amphibious.AmphibiousController`

## C

**CPG (Central Pattern Generator)**
A biological neural network capable of producing coordinated rhythmic output signals (like walking or swimming) without any rhythmic inputs. Modeled in FARMS as a system of coupled oscillators.
*Implementation*:
- Math: `farms_amphibious/control/ode.pyx`
- Integration Loop: `farms_amphibious.control.network.NetworkODE`
- Controller wrapper: `farms_amphibious.control.amphibious.AmphibiousController`

**ControlType**
An `IntEnum` defining the modes by which joints and actuators are driven. Values: `POSITION=0`, `VELOCITY=1`, `TORQUE=2`, `SPRINGREF=3`, `SPRINGCOEF=4`, `DAMPINGCOEF=5`, `MUSCLE=6`.
*Implementation*: `farms_core.model.control.ControlType`

## D

**dm_control**
Google DeepMind's software stack for physics-based simulation and Reinforcement Learning environments, built on MuJoCo. FARMS uses it instead of the raw MuJoCo bindings.
*Implementation*: The entire `farms_mujoco` backend is built around `dm_control.rl.control.Task` and `dm_control.rl.control.Environment`. See `farms_mujoco/simulation/task.py` and `farms_mujoco/simulation/simulation.py`.

**Descending Drive**
Top-down command signals that modulate CPG frequency and amplitude to steer the animat. Implemented via `DescendingDrive` ABC and `OrientationFollower`.
*Implementation*: `farms_amphibious.control.drive.DescendingDrive`

## E

**Ekeberg Muscle Model**
A phenomenological muscle model that translates neural excitation into joint torque, incorporating active force, passive stiffness, active stiffness, and damping.
*Implementation*: `farms_amphibious/control/ekeberg.pyx`

**ExperimentTask**
The core orchestrator of the physics loop in `farms_mujoco`. Inherits from `dm_control.rl.control.Task` and calls the extensions (`TaskExtension`) listed in the simulation and animat files.
*Implementation*: `farms_mujoco.simulation.task.ExperimentTask`

## M

**MJCF (MuJoCo XML Format)**
An XML format designed explicitly for the MuJoCo physics engine to define rigid body dynamics, joints, and actuators. FARMS dynamically generates MJCF files from user configurations.
*Implementation*: `farms_mujoco/simulation/mjcf.py`

**MuJoCo**
The underlying physics engine used by FARMS for rigid body dynamics simulation. Accessed via `dm_control`.
*Implementation*: `farms_mujoco.simulation.simulation.Simulation`

## N

**NetworkODE**
A concrete implementation of `AnimatNetwork` that integrates CPG oscillator ODEs using `scipy.integrate.ode` with the `dopri5` integrator.
*Implementation*: `farms_amphibious.control.network.NetworkODE`

## O

**Options**
Base class for all configuration objects. Implemented as a `dict` subclass with attribute-style access. Provides `load()` and `save()`.
*Implementation*: `farms_core.options.Options`

## P

**PositionMuscleCy**
A Cython actuator that converts CPG oscillator amplitude differences into joint position setpoints. Used when `motor.equation == 'position_muscle'`.
*Implementation*: `farms_amphibious.control.position_muscle_cy.PositionMuscleCy`

**PotentialMap**
Abstract base class defining mathematical potential fields for goal-directed locomotion. Concrete implementations: `StraightLinePotentialMap`, `CirclePotentialMap`, `EllipsoidPotentialMap`.
*Implementation*: `farms_amphibious.control.drive.PotentialMap`

## S

**SDF (Simulation Description Format)**
An XML format originally developed for the Gazebo simulator, used to describe objects and environments. FARMS uses SDF to define animat morphologies.
*Implementation*: Parsed via `farms_core.io.sdf`.

**SwimmingExtension**
An `AnimatExtension` that computes the fluid forces (buoyancy, drag, added mass) and applies them to MuJoCo via `physics.data.xfrc_applied`.
*Implementation*: `farms_mujoco.swimming.extension.SwimmingExtension`

**SwimmingHandler**
The Cython fluid force calculator used by `SwimmingExtension`: a single C loop over the links, run at every environment step.
*Implementation*: `farms_mujoco.swimming.hydrodynamics.SwimmingHandler`

## T

**TaskExtension**
The core plugin architecture of the FARMS simulation loop. Any custom logic (controllers, hydrodynamic forces, loggers) that needs to run per-timestep must inherit from this class.
*Implementation*: `farms_core.simulation.extensions.TaskExtension`

**Tegotae**
A Japanese concept translating roughly to "response" or "reaction". In FARMS CPG models, it refers to a specific form of sensory feedback where the CPG phase is modulated by the interaction of joint stretch and the current oscillator phase (θ_joint · sin(θ_cpg)).
*Implementation*: `farms_amphibious/control/ode.pyx`
