"""The nano duck's size against its servos' torque: python -I tools/nano_sweep.py [--stretch 1.0 1.15 1.3 ...] -> reports/nano/size_sweep.md

spec/nano_duck.py builds the smallest duck its SG90 modules allow (NANO_STRETCH=1). Each stretch makes the free lengths longer (shin, thigh, neck, body, head, feet: everything the servo
cases don't set) and the duck taller and heavier; this measures, with tools/checks.py's own statics and the parts' masses from tools/mass_model.py:
  - the knees in a crouch (both feet down: checks.py 'crouch'),
  - hip roll in single-leg stance, x 1.5 for the step's accelerations (checks.py 'single-leg stance (walking)'),
  - the neck and head pitch holding the head up (checks.py 'stand'),
each as a share of the servo's stall torque at about 5 V (spec/actuators.json: the lower published figure). The rule asked for: under about 50 % of stall.
A static estimate: no gait's dynamics beyond the 1.5, no friction, no servo's droop under load. Spec files are put back as they were for NANO_STRETCH=1.
"""
import argparse
import json
import os
import subprocess
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
LIMIT = 0.5


def measure():
    code = r"""
import json, sys
sys.path.insert(0, 'tools')
import checks as C, geom as G, mass_model as MM
s = G.load(); S = C.Solids(s); R = G.Robot(s); MS = C.Mass(s, S)
m = C.measure(s, S)
out = dict(height=m['height_m'], mass=sum(v[0] for v in MM.link_inertials(s, S).values()))
J = {j['name']: j for j in s['joints']}
stall = lambda n: s['actuators'][J[n]['act']]['peak_nm']
def tau(pose):
    q, support, cat, payload = C.POSES[pose]
    q = C.balance(s, R, MS, G.mirror_pose(q), support, payload)
    return C.statics(s, R, MS, q, support, payload)[0]
cr, sl, st = tau('crouch'), tau('single-leg stance (walking)'), tau('stand (microduck stance)')
out['knee_crouch'] = max(cr['L_knee'], cr['R_knee']) / stall('L_knee')
out['roll_single'] = 1.5 * max(sl['L_hip_roll'], sl['R_hip_roll']) / stall('L_hip_roll')
out['neck_stand'] = st['neck_pitch'] / stall('neck_pitch')
out['head_stand'] = st['head_pitch'] / stall('head_pitch')
out['knee_nm'], out['roll_nm'], out['neck_nm'] = max(cr['L_knee'], cr['R_knee']), max(sl['L_hip_roll'], sl['R_hip_roll']), st['neck_pitch']
print(json.dumps(out))
"""
    r = subprocess.run([sys.executable, '-c', code], cwd=ROOT, env=dict(os.environ, MILLI_RECIPE='nano'), capture_output=True, text=True, check=True)
    return json.loads(r.stdout.strip().splitlines()[-1])


def build(stretch):
    subprocess.run([sys.executable, '-I', 'spec/nano_duck.py'], cwd=ROOT, env=dict(os.environ, NANO_STRETCH=str(stretch)), capture_output=True, check=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--stretch', type=float, nargs='*', default=[1.0, 1.15, 1.3, 1.5, 1.75, 2.0])
    a = ap.parse_args()
    rows = []
    try:
        for k in a.stretch:
            build(k)
            m = measure()
            rows.append((k, m))
            print(f"stretch {k:.2f}: {m['height'] * 100:.1f} cm, {m['mass'] * 1000:.0f} g; knee crouch {m['knee_crouch'] * 100:.0f} %, hip roll single-leg x 1.5 {m['roll_single'] * 100:.0f} %, "
                  f"neck {m['neck_stand'] * 100:.0f} %, head pitch {m['head_stand'] * 100:.0f} % of stall", flush=True)
    finally:
        build(1.0)
    L = ['# The nano duck: size against torque', '', 'From `python -I tools/nano_sweep.py`. Each row is spec/nano_duck.py with its free lengths stretched (`NANO_STRETCH`); the servo cases set the rest.',
         'Torque as a share of stall at about 5 V (SG90 0.157 N m, MG90S 0.177 N m: spec/actuators.json), from tools/checks.py\'s statics. The rule: under about 50 %.', '',
         '| stretch | height (cm) | mass (g) | knees, crouch | hip roll, single leg x 1.5 | neck, standing | head pitch, standing |', '|---|---|---|---|---|---|---|']
    for k, m in rows:
        f = lambda v: f"{v * 100:.0f} %" + (' **over**' if v > LIMIT else '')
        L.append(f"| {k:.2f} | {m['height'] * 100:.1f} | {m['mass'] * 1000:.0f} | {f(m['knee_crouch'])} ({m['knee_nm']:.3f} N m) | {f(m['roll_single'])} ({m['roll_nm']:.3f} N m x 1.5) | "
                 f"{f(m['neck_stand'])} ({m['neck_nm']:.3f} N m) | {f(m['head_stand'])} |")
    L += ['', 'Heights under the first row\'s cannot be built in this layout: see spec/nano_duck.py\'s docstring. Only the first row is the checked design (tools/checks.py, assembly.py, buildable.py, printable.py at 0 failing);',
          'the stretched rows are torque estimates, their shapes not checked, and their thin members would need thickening, which adds mass. A static estimate only: walking adds the dynamics, and a real SG90 sags under load before it stalls.',
          '', 'Chosen: stretch 1.0. The rule alone (the largest under 50 %) points to about 27 cm, microduck\'s own size, which isn\'t a palm-sized duck; 12-18 cm, the range asked for, is under what this layout can build, so the duck is the smallest it can.']
    out = os.path.join(ROOT, 'reports', 'nano', 'size_sweep.md')
    os.makedirs(os.path.dirname(out), exist_ok=True)
    open(out, 'w', encoding='utf-8').write('\n'.join(L) + '\n')
    print(out)
    return 0


if __name__ == '__main__':
    sys.exit(main())
