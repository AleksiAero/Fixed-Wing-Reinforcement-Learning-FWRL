"""Build the selected course, open Blender and start fresh live learning."""
import argparse
import subprocess
import time
from pathlib import Path
from .world import load_world


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--world',default='worlds/mountain_gauntlet.json')
    parser.add_argument('--steps',type=int,default=10000000)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    world_path = Path(args.world).resolve()
    if load_world(world_path).get('control_mode') not in ('direct','elevons'):
        raise ValueError('Live learning from zero requires a direct-control world')
    output = root/'worlds'/'mountain_gauntlet.blend'
    # Every build uses the selected JSON; no trajectory or checkpoint is loaded.
    subprocess.run(['blender','--background','--python','blender/build_world.py','--',
                    '--world',str(world_path),'--output',str(output),'--learning-steps',str(args.steps)],cwd=root,check=True)
    logdir = root/'runs'/'live'
    logdir.mkdir(parents=True,exist_ok=True)
    with (logdir/f'blender-{time.time_ns()}.log').open('w') as log:
        subprocess.Popen(['blender',str(output),'--python','blender/start_learning.py'],
                         cwd=root,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    print('Blender opened: live training from zero. Use Flight Lab sidebar to pause or stop.',flush=True)


if __name__ == '__main__':
    main()
