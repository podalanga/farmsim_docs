#!/usr/bin/env python3
"""Generate the figures of the documentation from AmphiBot simulations.

    MUJOCO_GL=egl python tools/make_figures.py

Runs the AmphiBot example headless (land to water, 10 s), replays the log
offscreen with MuJoCo, and writes to docs/assets/figures/:

- amphibot-model.png: the robot
- amphibot-land-to-water.gif: crawling, ramp, swimming
- amphibot-swim.gif: swimming, closer
- amphibot-trajectory.png, amphibot-joints.png, amphibot-water.png: plots
  of examples/amphibot/plot_run.py

The figures are committed: rerun this script when the example changes.
Needs the FARMS packages, imageio and matplotlib (offscreen rendering:
MUJOCO_GL=egl on a GPU, MUJOCO_GL=osmesa on the CPU).
"""

import argparse
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np
import h5py
import imageio
import mujoco

HERE = Path(__file__).resolve().parent
DOCS = HERE.parent / 'docs'
EXAMPLE = HERE.parent.parent / 'examples' / 'amphibot'
LINKS = 'FARMSLISTanimats/0/sensors/links'
JOINTS = 'FARMSLISTanimats/0/sensors/joints'


def run_example(workdir, n_iterations):
    """Copy the example, run it headless and return its folder"""
    folder = Path(workdir) / 'amphibot'
    shutil.copytree(EXAMPLE, folder, ignore=shutil.ignore_patterns(
        'Output', 'cob_lut_cache', '__pycache__',
    ))
    config = folder / 'simulation_config.yaml'
    text = config.read_text()
    for key, value in [
            ('n_iterations', n_iterations),
            ('buffer_size', n_iterations),
            ('headless', 'true'),
            ('show_progress', 'false'),
    ]:
        lines = [
            f'{line.split(":")[0]}: {value}'
            if line.strip().startswith(f'{key}:') else line
            for line in text.splitlines()
        ]
        text = '\n'.join(lines) + '\n'
    config.write_text(text)
    subprocess.run(
        [sys.executable, 'run_sim.py', '--experiment_config', 'experiment_config.yaml'],
        cwd=folder, check=True, stdout=subprocess.DEVNULL,
    )
    return folder


class Replay:
    """Offscreen replay of a logged run"""

    def __init__(self, folder, width, height):
        self.model = mujoco.MjModel.from_xml_path(
            str(folder / 'Output' / 'simulation_mjcf.xml'),
        )
        self.data = mujoco.MjData(self.model)
        self.model.vis.global_.offwidth = max(self.model.vis.global_.offwidth, width)
        self.model.vis.global_.offheight = max(self.model.vis.global_.offheight, height)
        self.renderer = mujoco.Renderer(self.model, height=height, width=width)
        with h5py.File(folder / 'Output' / 'simulation.hdf5', 'r') as log:
            self.times = log['times'][()]
            self.links = log[f'{LINKS}/array'][()]
            self.joints = log[f'{JOINTS}/array'][()]
            links_names = [name.decode() for name in log[f'{LINKS}/names'][()]]
            joints_names = [name.decode() for name in log[f'{JOINTS}/names'][()]]
        from farms_core.sensors.sensor_convention import sc
        self.sc = sc
        self.head = links_names.index('head')
        # qpos address of each logged joint (MJCF names are prefixed a0_)
        self.joints_qpos = [
            (joint_i, self.model.jnt_qposadr[self.model.joint(f'a0_{name}').id])
            for joint_i, name in enumerate(joints_names)
        ]
        self.root_qpos = self.model.jnt_qposadr[self.model.joint('a0_root_amphibot').id]
        self.camera = mujoco.MjvCamera()
        self.camera.type = mujoco.mjtCamera.mjCAMERA_FREE
        # Visual geoms only (group 1), not the collision geoms (group 2)
        self.options = mujoco.MjvOption()
        self.options.geomgroup[2] = 0

    def set_state(self, iteration):
        """Robot pose of a logged iteration"""
        sc = self.sc
        link = self.links[iteration, self.head]
        position = link[sc.link_urdf_position_x:sc.link_urdf_position_z+1]
        x, y, z, w = link[sc.link_urdf_orientation_x:sc.link_urdf_orientation_w+1]
        self.data.qpos[self.root_qpos:self.root_qpos+3] = position
        self.data.qpos[self.root_qpos+3:self.root_qpos+7] = [w, x, y, z]
        for joint_i, address in self.joints_qpos:
            self.data.qpos[address] = self.joints[iteration, joint_i, sc.joint_position]
        mujoco.mj_forward(self.model, self.data)

    def head_position(self, iteration):
        """Logged head position"""
        sc = self.sc
        return self.links[iteration, self.head, sc.link_urdf_position_x:sc.link_urdf_position_z+1]

    def render(self, iteration, distance, azimuth, elevation, lookat=None):
        """Image of a logged iteration"""
        self.set_state(iteration)
        self.camera.lookat[:] = (
            self.head_position(iteration) - [0.25, 0, 0]
            if lookat is None else lookat
        )
        self.camera.distance = distance
        self.camera.azimuth = azimuth
        self.camera.elevation = elevation
        self.renderer.update_scene(self.data, camera=self.camera, scene_option=self.options)
        return self.renderer.render()

    def iteration_at(self, time):
        """Logged iteration closest to a time"""
        return int(np.argmin(np.abs(self.times - time)))


def gif(path, frames, fps):
    """Save frames as a looping GIF"""
    imageio.mimsave(path, frames, duration=1000/fps, loop=0)
    print(f'{path.name}: {len(frames)} frames, {path.stat().st_size/1e6:.1f} MB')


def main():
    """Run, replay and save the figures"""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--out', default=str(DOCS / 'assets' / 'figures'))
    parser.add_argument('--keep', help='Keep the simulation folder here')
    args = parser.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    workdir = args.keep or tempfile.mkdtemp(prefix='farms-figures-')
    folder = run_example(workdir, n_iterations=10001)
    replay = Replay(folder, width=640, height=360)

    # Still of the robot on land
    image = replay.render(replay.iteration_at(1.5), distance=0.6, azimuth=140, elevation=-30)
    imageio.imwrite(out / 'amphibot-model.png', image)
    print('amphibot-model.png')

    # Land to water, following the head: 10 s at 2x speed, 12.5 frames/s
    frames = [
        replay.render(replay.iteration_at(time), distance=1.6, azimuth=120, elevation=-25)[::2, ::2]
        for time in np.arange(0, replay.times[-1], 0.16)
    ]
    gif(out / 'amphibot-land-to-water.gif', frames, fps=12.5)

    # Swimming, closer, from the side: 3 s in real time
    frames = [
        replay.render(replay.iteration_at(time), distance=0.9, azimuth=90, elevation=-15)[::2, ::2]
        for time in np.arange(6.5, 9.5, 0.08)
    ]
    gif(out / 'amphibot-swim.gif', frames, fps=12.5)

    # Plots
    plots = folder / 'Output' / 'plots'
    subprocess.run(
        [sys.executable, 'plot_run.py', '--out', str(plots)],
        cwd=folder, check=True, stdout=subprocess.DEVNULL,
    )
    for name in ('trajectory', 'joints', 'water'):
        shutil.copy(plots / f'{name}.png', out / f'amphibot-{name}.png')
        print(f'amphibot-{name}.png')
    if not args.keep:
        shutil.rmtree(workdir)


if __name__ == '__main__':
    main()
