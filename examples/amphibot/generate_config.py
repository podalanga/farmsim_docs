#!/usr/bin/env python3
"""Generate animat_config.yaml for the AmphiBot example.

The animat options (links, joints, sensors, motors, CPG network) are built by
farms_amphibious from the morphology and a few gait parameters, then saved
as YAML:

    python generate_config.py

Edit the parameters below and rerun this script instead of editing the
generated animat_config.yaml by hand.
"""

from copy import deepcopy

import numpy as np

from farms_amphibious.model.options import AmphibiousOptions

N_JOINTS = 7  # Yaw joints between the head and the 7 body segments
BODY = ['head'] + [f'seg{i}' for i in range(1, N_JOINTS + 1)]
WHEELS = [f'{link}_wheel' for link in BODY]  # One passive wheel per link
LINKS = BODY + WHEELS
JOINTS = (
    [f'joint{i}' for i in range(1, N_JOINTS + 1)]
    + [f'{wheel}_joint' for wheel in WHEELS]
)

# Gait: a travelling wave of 0.8 body lengths at 1 Hz (anguilliform swimming)
DRIVE = 4.0  # Initial descending drive, within the oscillators' [1, 5] range
FREQUENCY = 1.0  # [Hz] at DRIVE
WAVES = 0.8  # Number of waves along the body

# Quadratic drag per link [N/(m/s)^2], negative (opposing the motion), along
# the link axes: x along the body, y sideways, z vertical. Sideways drag is
# about 6 times the axial drag, and twice as large on the tail segment (fin).
DRAG_BODY = [[-0.25, -1.5, -1.8], [0, 0, 0]]
DRAG_TAIL = [[-0.25, -3.0, -1.8], [0, 0, 0]]
DRAG_WHEEL = [[0, 0, 0], [0, 0, 0]]


def build(controller):
    """Animat options with the given controller extension"""
    options = AmphibiousOptions.from_options({
        'sdf_path': 'models/amphibot.sdf',
        'n_joints_body': N_JOINTS,
        'n_dof_legs': 0,
        'n_legs': 0,
        'n_joints_passive': len(WHEELS),
        'links_names': LINKS,
        'joints_names': JOINTS,
        'spawn_position': [-1.0, 0, 0.075],  # On land, facing the water (+x)
        # Links: friction and water drag
        'default_lateral_friction': 0.5,
        # (copies, so that the YAML file has no anchors and aliases)
        'drag_coefficients': [
            deepcopy(drag)
            for drag in [DRAG_BODY]*N_JOINTS + [DRAG_TAIL] + [DRAG_WHEEL]*len(WHEELS)
        ],
        # Free-rolling wheels: [joint, stiffness, damping, friction]
        'joints_passive': [
            [f'{wheel}_joint', 0, 1e-5, 0] for wheel in WHEELS
        ],
        # CPG: one left/right oscillator pair per joint
        'drives_init': [DRIVE, DRIVE],
        'body_freq_gain': 2*np.pi*FREQUENCY/DRIVE,
        'body_osc_gain': 0.15,
        'body_walk_amplitude': 1.0,
        'weight_osc_body_side': 30.0,
        'weight_osc_body_down': 30.0,
        'body_phase_bias': 2*np.pi*WAVES/N_JOINTS,
        # Motors: position control of each joint from the oscillators
        'default_max_torque': 2.0,  # [Nm], AmphiBot joint effort limit
        'motor_gains': [[1.0, 0.01, 0] for _ in JOINTS],  # Unused by the wheels
        'muscle_alpha': 0.5,
        'muscle_beta': 1.0,
        'muscle_gamma': 0.1,
        'muscle_delta': 0.001,
        'extensions': [
            # The controller and the water forces run as animat extensions
            controller,
            {
                'loader': 'farms_mujoco.swimming.extension.SwimmingExtension',
                'config': {'water_properties': None},
            },
        ],
    })
    options.control.controller_loader = controller['loader']
    return options


def main():
    """Build and save the animat options"""
    # Built-in CPG network (farms_amphibious)
    build({
        'loader': 'farms_amphibious.control.amphibious.AmphibiousController',
        'config': {},
    }).save('animat_config.yaml')
    # Custom travelling-wave controller (controller/traveling_wave.py)
    build({
        'loader': 'controller.traveling_wave.TravelingWaveController',
        'config': {'mode': 'auto', 'turn': 0.0},
    }).save('animat_config_wave.yaml')
    print('Saved animat_config.yaml and animat_config_wave.yaml')


if __name__ == '__main__':
    main()
