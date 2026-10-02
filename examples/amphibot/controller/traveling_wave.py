"""Travelling-wave gait for AmphiBot, with land/water switching.

    angle_i(t) = amp_i * sin(phase(t) - i * dphi) + turn

The wave travels from head to tail, which pushes the body forward.
- land: serpentine crawling (constant amplitude, slower)
- water: anguilliform swimming (amplitude growing toward the tail, faster)

The gait switches when the head crosses the water surface (z = 0), with a
hysteresis band, and blends smoothly between the two gaits. This is a
FARMS port of the gait_controller.py of the Gazebo AmphiBot example.
"""

import numpy as np

from farms_core.model.control import AnimatController, ControlType

GAITS = {
    'land': {'freq': 0.6, 'amp_head': 0.45, 'amp_tail': 0.45, 'waves': 1.0},
    'water': {'freq': 1.0, 'amp_head': 0.20, 'amp_tail': 0.55, 'waves': 0.8},
}
WATER_ENTER_Z, WATER_EXIT_Z = 0.015, 0.035  # [m] hysteresis band
BLEND_TAU = 1.0  # [s] time constant of the blend between gaits


class TravelingWaveController(AnimatController):
    """Open-loop travelling wave with land/water gait switching"""

    def __init__(
            self,
            animat_i: int,
            joints_names: tuple,
            max_torques: tuple,
            links_sensors,
            head_index: int,
            mode: str = 'auto',
            turn: float = 0.0,
    ):
        super().__init__(
            animat_i=animat_i,
            joints_names=joints_names,
            muscles_names=[],
            max_torques=max_torques,
        )
        assert mode in ('auto', 'land', 'water'), mode
        self.links = links_sensors
        self.head_index = head_index
        self.mode = mode
        self.turn = turn
        self.medium = 'land' if mode == 'auto' else mode
        self.blend = 0.0 if self.medium == 'land' else 1.0  # 0: land, 1: water
        self.phase = 0.0

    @classmethod
    def from_options(
            cls, config, experiment_options, animat_i, animat_data,
            animat_options,
    ):
        """Controller from the animat options (config: the YAML `config:`)"""
        # Only the body joints are commanded, the wheel joints roll freely
        motors = [
            motor for motor in animat_options.control.motors
            if not motor.passive.is_passive
        ]
        names = [motor.joint_name for motor in motors]
        control_types = {
            motor.joint_name: ControlType.from_string_list(motor.control_types)
            for motor in motors
        }
        links_names = animat_data.sensors.links.names
        return cls(
            animat_i=animat_i,
            joints_names=cls.joints_from_control_types(
                joints_names=names,
                joints_control_types=control_types,
            ),
            max_torques=cls.max_torques_from_control_types(
                joints_names=names,
                max_torques={
                    motor.joint_name: motor.limits_torque[1]
                    for motor in motors
                },
                joints_control_types=control_types,
            ),
            links_sensors=animat_data.sensors.links,
            head_index=list(links_names).index(config.get('head', 'head')),
            mode=config.get('mode', 'auto'),
            turn=config.get('turn', 0.0),
        )

    def update_medium(self, iteration: int):
        """Switch gait when the head crosses the water surface"""
        if self.mode != 'auto':
            return
        z = self.links.com_position(max(iteration-1, 0), self.head_index)[2]
        if self.medium == 'land' and z < WATER_ENTER_Z:
            self.medium = 'water'
        elif self.medium == 'water' and z > WATER_EXIT_Z:
            self.medium = 'land'

    def positions(
            self, iteration: int, time: float, timestep: float,
    ) -> dict[str, float]:
        """Joint position targets of this control step"""
        self.update_medium(iteration)
        target = 1.0 if self.medium == 'water' else 0.0
        self.blend += (target - self.blend)*min(1.0, timestep/BLEND_TAU)
        gait = {
            key: (1 - self.blend)*GAITS['land'][key] + self.blend*GAITS['water'][key]
            for key in GAITS['land']
        }
        self.phase += 2*np.pi*gait['freq']*timestep
        joints = self.joints_names[ControlType.POSITION]
        n_joints = len(joints)
        dphi = 2*np.pi*gait['waves']/n_joints
        return {
            joint: (
                (
                    gait['amp_head']
                    + (gait['amp_tail'] - gait['amp_head'])*i/max(1, n_joints-1)
                )*np.sin(self.phase - i*dphi)
                + self.turn
            )
            for i, joint in enumerate(joints)
        }
