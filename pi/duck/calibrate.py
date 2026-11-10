"""Measure each servo's centre tick on the real robot, with its torque off, posed by hand at the model's zero pose: legs straight, head level, beak shut.

  python -m duck.calibrate --port /dev/serial0

Saves the centres into the config. Directions aren't measured here: run `python run.py --policy wave --no-fall` next, on a stand, and fix a joint's `direction` if it moves the wrong way.
"""
import argparse
import json
import os

from . import dxl
from .hardware import load_config

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

ap = argparse.ArgumentParser()
ap.add_argument('--port', default='/dev/serial0')
ap.add_argument('--config', default=os.path.join(HERE, 'config', 'dynamixel.json'))
a = ap.parse_args()
cfg = load_config(a.config)
bus = dxl.Bus(a.port)
for name, j in cfg['joints'].items():
    bus.write(j['id'], dxl.TORQUE_ENABLE, 0)
input('Pose the duck at zero: legs straight, head level, beak shut. Then press Enter. ')
for name, j in cfg['joints'].items():
    j['centre'] = bus.read(j['id'], dxl.PRESENT_POSITION)
    print(f"{name:18s} id {j['id']:2d} centre {j['centre']}")
with open(a.config, 'w', encoding='utf-8') as f:
    json.dump(cfg, f, indent=1)
print('saved', a.config)
