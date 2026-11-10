"""The Pi's config and the simulated Pi's model, from the spec.

  MILLI_RECIPE=dynamixel python tools/make_pi_config.py

Writes pi/config/dynamixel.json, and pi/sim/model/ through tools/make_rl_model.py --contact: the whole body collides, as a real one does when it falls. Servo ids: left leg 1-5, right leg 6-10, neck to beak 11-15, hip to ankle in each leg.
Directions start at +1 and centres at mid-range: python -m duck.calibrate measures the centres, and a real robot may need a direction flipped.
"""
import json
import os
import re
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
recipe = os.environ.get('MILLI_RECIPE', 'dynamixel')
model = os.path.join(ROOT, 'pi', 'sim', 'model')
subprocess.run([sys.executable, os.path.join(ROOT, 'tools', 'make_rl_model.py'), model, '--contact'], check=True, env=dict(os.environ, MILLI_RECIPE=recipe))
xml = open(os.path.join(model, 'milli.xml'), encoding='utf-8').read()
home = json.load(open(os.path.join(model, 'home.json'), encoding='utf-8'))
joints = re.findall(r'<joint name="(\w+)" axis="[^"]+" range="([-\d.]+) ([-\d.]+)"', xml)
order = [j[0] for j in joints]
assert len(order) == 14, order
ids = {}
for k, n in enumerate(('left_hip_yaw', 'left_hip_roll', 'left_hip_pitch', 'left_knee', 'left_ankle', 'right_hip_yaw', 'right_hip_roll', 'right_hip_pitch', 'right_knee', 'right_ankle', 'neck_pitch', 'head_pitch', 'head_yaw', 'head_roll', 'beak')):
    ids[n] = k + 1
servo = next(iter(home['servos'].values()))
cfg = {
    'recipe': recipe,
    'policy_joints': order,
    'extra_joints': ['beak'],
    'joints': {n: dict(id=i, direction=1, centre=2048) for n, i in ids.items()},
    'home': home['joint_pos'],
    'limits': {j[0]: [float(j[1]), float(j[2])] for j in joints},
    'root_z': home['root_z'],
    'position_p': 200,
    'servo': dict(kp=servo['kp'], peak_nm=servo['peak_nm'], noload_rads=servo['noload_rads']),
    'imu_to_robot': [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
}
os.makedirs(os.path.join(ROOT, 'pi', 'config'), exist_ok=True)
with open(os.path.join(ROOT, 'pi', 'config', f'{recipe}.json'), 'w', encoding='utf-8') as f:
    json.dump(cfg, f, indent=1)
print(f'{len(order)} policy joints -> pi/config/{recipe}.json, pi/sim/model')
