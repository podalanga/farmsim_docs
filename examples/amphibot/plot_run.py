#!/usr/bin/env python3
"""Plot a recorded AmphiBot run.

    python plot_run.py [--log Output/simulation.hdf5] [--out Output/plots]

Saves trajectory.png, joints.png and water.png, and prints the speeds on
land and in water.
"""

import argparse
import os

import numpy as np
import matplotlib
matplotlib.use('Agg')  # Write files, no window
import matplotlib.pyplot as plt  # noqa: E402

from farms_core.experiment.data import ExperimentData  # noqa: E402
from farms_core.sensors.sensor_convention import sc  # noqa: E402

BODY_JOINTS = [f'joint{i}' for i in range(1, 8)]


def main():
    """Load the log and plot it"""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--log', default='Output/simulation.hdf5')
    parser.add_argument('--out', default='Output/plots')
    args = parser.parse_args()
    os.makedirs(args.out, exist_ok=True)

    # Load the data
    data = ExperimentData.from_file(args.log)
    times = np.asarray(data.times)
    links = data.animats[0].sensors.links
    joints = data.animats[0].sensors.joints
    xfrc = data.animats[0].sensors.xfrc
    links_names = list(links.names)
    joints_names = list(joints.names)
    com = np.asarray(links.array)[:, :, sc.link_com_position_x:sc.link_com_position_z+1]
    positions = np.asarray(joints.array)[:, :, sc.joint_position]
    forces = np.asarray(xfrc.array)[:, :, sc.xfrc_force_x:sc.xfrc_force_z+1]
    head = com[:, links_names.index('head')]
    in_water = head[:, 2] < 0

    # Speeds on land and in water (head velocity along x)
    speed = np.gradient(head[:, 0], times)
    for medium, mask in [('on land', ~in_water), ('in water', in_water)]:
        if mask.sum() > 100:
            print(f'Mean speed {medium}: {speed[mask].mean():.2f} m/s')

    # Trajectory seen from above, coloured by medium
    fig, ax = plt.subplots(figsize=(8, 3))
    ax.plot(head[~in_water, 0], head[~in_water, 1], '.', ms=1, label='On land')
    ax.plot(head[in_water, 0], head[in_water, 1], '.', ms=1, label='In water')
    ax.axvline(0, color='grey', ls='--', lw=1, label='Ramp start')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_title('Head trajectory')
    ax.legend(markerscale=10)
    ax.set_aspect('equal', adjustable='datalim')
    fig.tight_layout()
    fig.savefig(os.path.join(args.out, 'trajectory.png'), dpi=150)

    # Body joint angles: the travelling wave
    fig, ax = plt.subplots(figsize=(8, 4))
    for name in BODY_JOINTS:
        ax.plot(times, positions[:, joints_names.index(name)], lw=1, label=name)
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Joint angle [rad]')
    ax.set_title('Body joints')
    ax.legend(ncol=7, fontsize='small', loc='upper center')
    fig.tight_layout()
    fig.savefig(os.path.join(args.out, 'joints.png'), dpi=150)

    # Head height and vertical water force on the body
    body = [links_names.index(name) for name in ['head'] + [f'seg{i}' for i in range(1, 8)]]
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 5), sharex=True)
    ax1.plot(times, head[:, 2])
    ax1.axhline(0, color='grey', ls='--', lw=1)
    ax1.set_ylabel('Head height [m]')
    ax2.plot(times, forces[:, body, 2].sum(axis=1))
    ax2.set_ylabel('Vertical fluid force [N]')
    ax2.set_xlabel('Time [s]')
    ax1.set_title('Entering the water')
    fig.tight_layout()
    fig.savefig(os.path.join(args.out, 'water.png'), dpi=150)
    print(f'Plots saved in {args.out}/')


if __name__ == '__main__':
    main()
