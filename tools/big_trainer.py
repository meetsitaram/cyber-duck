"""The big duck's models for the trainer, and the trainer.json that registers its tasks (Mjlab-Velocity-Flat-Milli-Big and the rest).

  python tools/big_trainer.py ../microduck_rl/src/mjlab_microduck/robot

Plain python (MuJoCo). Writes <trainer>/milli_big and <trainer>/milli_big_contact. Only the walk is meant for it: its spec says it can't pick itself up off the
floor, so its StandUp and Roulade tasks register but aren't expected to learn.
"""
import json
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import new_duck as N  # noqa: E402


def main():
    trainer = os.path.abspath(sys.argv[1]) if len(sys.argv) > 1 else os.path.join(ROOT, '..', 'microduck_rl', 'src', 'mjlab_microduck', 'robot')
    if not os.path.isdir(trainer):
        sys.exit(f'no trainer robot folder at {trainer}: clone and patch the trainer first (docs/TRAINING.md)')
    env = dict(os.environ, MILLI_RECIPE='big', PYTHONIOENCODING='utf-8')
    for name, extra in (('milli_big', []), ('milli_big_contact', ['--contact'])):
        subprocess.run([sys.executable, os.path.join(HERE, 'make_rl_model.py'), os.path.join(trainer, name)] + extra, env=env, cwd=ROOT, check=True)
    height = N.measure('big')['height']                 # as tools/new_duck.py measures every duck: the trainer scales by height / microduck's 0.264 m
    lie, sit, stand = N.clearances('big', trainer, top=2.0)
    home = json.load(open(os.path.join(trainer, 'milli_big', 'home.json')))
    out = dict(height=height / 0.264, stand=stand - 0.001, prone=[lie + 0.002, lie + 0.025], sit=[sit + 0.002, sit + 0.02])
    json.dump(out, open(os.path.join(trainer, 'milli_big_contact', 'trainer.json'), 'w'), indent=1)
    print(f"mass {home.get('mass_kg')} kg; trainer.json {out}")
    print('tasks: Mjlab-Velocity-Flat-Milli-Big (the walk), Mjlab-StandUp-Flat-Milli-Big, Mjlab-Roulade-Flat-Milli-Big')


if __name__ == '__main__':
    main()
