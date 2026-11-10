"""A duck at a height you name: python tools/new_duck.py --height-cm 34 [--servo HD1910_C001] [--name d34] [--compare] [--finish]

Starts from the nearest servo recipe in spec/robot.py, finds the leg and body scale K that gives the height (the head follows it, but never below the size
that holds its servos), writes spec/recipes/<name>.json, generates the spec and runs tools/checks.py. What this can't do it says: a servo's case sets a smallest
duck, and the checks' failures need the design changing by hand (the droid-from-spec skill's duck-at-size section lists which number does what).

  --compare    try every servo in the catalog at this height, and print which reach it, their mass, strength and failing checks
  --finish     once the checks pass: measure the height each reset pose clears the floor, and write the trainer's models and its trainer.json, so its tasks register
  --trainer D  the trainer's robot folder (default ../milli-rl/src/mjlab_microduck/robot). Plain python.
"""
import argparse
import json
import os
import re
import subprocess
import sys

import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
SPEC = os.path.join(ROOT, 'spec')
# the servo, the recipe it starts from, and each template's smallest K and head: below those, parts stop fitting (found round by round in spec/robot.py)
CATALOG = {'XL330_M288': 'dynamixel', 'HD1910_C001': 'hd1910', 'STS3215_C044': 'feetech', 'STS3215_C001': 'feetech'}
TUCK = {'left_hip_pitch': -0.8, 'left_knee': 0.95, 'left_ankle': 1.05, 'neck_pitch': -1.0, 'head_pitch': 1.0, 'right_hip_pitch': 0.8, 'right_knee': -0.95, 'right_ankle': -1.05}
SIT = {'left_hip_pitch': -0.45, 'left_knee': 0.95, 'left_ankle': 0.0, 'right_hip_pitch': 0.45, 'right_knee': -0.95, 'right_ankle': 0.0}
# what a roll has needed: stall over the torque to hold the tucked body on the head. Microduck, at the stall its own trainer models: 2.1. The hybrid's scripted roll
# went over between 0.72 and 0.93 (tools/flip_search.py); it failed at 0.40
ROLL_MARGIN = 0.93


def run(args, env=None, check=True):
    return subprocess.run([sys.executable] + args, cwd=ROOT, env=dict(os.environ, **(env or {})), capture_output=True, text=True, check=check)


def recipes():
    code = "import json,sys; sys.path.insert(0,'spec'); import robot; print(json.dumps(robot.RECIPES))"
    return json.loads(run(['-c', code]).stdout.strip().splitlines()[-1])


def write(name, rec):
    os.makedirs(os.path.join(SPEC, 'recipes'), exist_ok=True)
    json.dump(rec, open(os.path.join(SPEC, 'recipes', name + '.json'), 'w'), indent=1)
    run(['-I', 'spec/robot.py'], {'MILLI_RECIPE': name})


def measure(name):
    code = ("import json,sys; sys.path.insert(0,'tools'); import checks as C, geom as G, mass_model as MM; s=G.load(); S=C.Solids(s); m=C.measure(s,S);"
            "print(json.dumps(dict(height=m['height_m'],width=m['overall_width_m'],mass=sum(v[0] for v in MM.link_inertials(s,S).values()))))")
    return json.loads(run(['-c', code], {'MILLI_RECIPE': name}).stdout.strip().splitlines()[-1])


def make(servo, K, base, height, head_servo=None):
    r = json.loads(json.dumps(base))
    r['K'], r['HK'] = K, max(base['HK'], height / 0.264)                 # microduck's head at this height, and never smaller than holds its servos
    if servo.startswith('STS'):
        r.update(leg=servo)
    else:
        r.update(yaw=servo, leg=servo, neck=servo, head=servo, head2=servo)
    if head_servo:
        r.update(neck=head_servo, head=head_servo, head2=head_servo)
    return r


def fit(name, servo, height, base, head_servo=None):
    """Bisect K for the height; K can't go below the template's, whose layout was worked out with its parts just fitting."""
    lo, hi = base['K'], base['K'] * 4
    write(name, make(servo, lo, base, height, head_servo))
    least = measure(name)
    if height <= least['height']:
        return lo, least, least['height'] - height > 0.003
    for _ in range(9):
        mid = (lo + hi) / 2
        write(name, make(servo, mid, base, height, head_servo))
        if measure(name)['height'] < height:
            lo = mid
        else:
            hi = mid
    write(name, make(servo, hi, base, height, head_servo))
    return hi, measure(name), False


def checks(name):
    out = os.path.join('reports', name)
    os.makedirs(os.path.join(ROOT, out), exist_ok=True)
    r = run(['-I', 'tools/checks.py', '--out', os.path.join(out, 'checks.md')], {'MILLI_RECIPE': name}, check=False)
    return [ln for ln in r.stdout.splitlines() if ln.startswith('FAIL')]


def trainer_models(name, trainer):
    env = {'MILLI_RECIPE': name}
    run(['tools/make_rl_model.py', os.path.join(trainer, f'milli_{name}')], env)
    run(['tools/make_rl_model.py', os.path.join(trainer, f'milli_{name}_contact'), '--contact'], env)


def margin(name, trainer):
    """Stall over the torque to hold the tucked body on the head, at head pitch; 0 for the servo it's on."""
    import mujoco
    M = mujoco.MjModel.from_xml_path(os.path.join(trainer, f'milli_{name}_contact', 'milli.xml'))
    D = mujoco.MjData(M)
    home = json.load(open(os.path.join(trainer, f'milli_{name}_contact', 'home.json')))
    D.qpos[:] = M.qpos0
    D.qpos[2] = 1.0
    for k, v in TUCK.items():
        j = M.joint(k)
        D.qpos[j.qposadr[0]] = np.clip(v, *M.jnt_range[j.id])
    mujoco.mj_forward(M, D)
    j = M.joint('head_pitch')
    kids = [int(M.jnt_bodyid[j.id])]
    for b in range(M.nbody):
        if M.body_parentid[b] in kids and b not in kids:
            kids.append(b)
    body = [b for b in range(1, M.nbody) if b not in kids]
    mass = M.body_mass[body]
    c = (mass[:, None] * D.xipos[body]).sum(0) / mass.sum()
    r = c - D.xanchor[j.id]
    lever = np.linalg.norm(r - D.xaxis[j.id] * (r @ D.xaxis[j.id]))
    stall = next(s['peak_nm'] for s in home['servos'].values() if 'head_pitch' in s['joints'])
    return stall / (mass.sum() * 9.81 * lever), stall


def clearances(name, trainer, top=0.6):
    """The root height at which each reset pose just clears the floor: lower, and the overlap throws the robot up for free. top: where the search starts, above the tallest pose (0.6 m for a small duck; the big duck needs more)."""
    import mujoco
    folder = os.path.join(trainer, f'milli_{name}_contact')
    x = open(os.path.join(folder, 'milli.xml')).read().replace('<worldbody>', '<worldbody><geom name="floor" type="plane" size="5 5 .1"/>', 1)
    p = os.path.join(folder, '_clear.xml')
    open(p, 'w').write(x)
    M = mujoco.MjModel.from_xml_path(p)
    os.remove(p)
    D = mujoco.MjData(M)
    home = json.load(open(os.path.join(folder, 'home.json')))
    f = M.geom('floor').id
    s = 2 ** -0.5

    def clear(quat, extra=None):
        D.qpos[:] = 0
        D.qpos[3:7] = quat
        for k, v in {**home['joint_pos'], **(extra or {})}.items():
            D.qpos[M.joint(k).qposadr[0]] = v
        for z in np.arange(top, 0.0, -0.002):
            D.qpos[2] = z
            mujoco.mj_forward(M, D)
            if any(f in (D.contact[i].geom1, D.contact[i].geom2) and D.contact[i].dist < 0 for i in range(D.ncon)):
                return float(z)
        return 0.0
    lying = [clear([s, 0, s, 0]), clear([s, 0, -s, 0])]
    for t in (-90, -60, -30, 30, 60, 90):
        q = np.zeros(4)
        mujoco.mju_mulQuat(q, np.array([s, 0, -s, 0]), np.array([np.cos(np.radians(t) / 2), 0, 0, np.sin(np.radians(t) / 2)]))
        lying.append(clear(q))
    return max(lying), clear([1, 0, 0, 0], SIT), home['root_z']


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--height-cm', type=float, required=True)
    ap.add_argument('--servo')
    ap.add_argument('--head-servo', help='a different servo for the head and neck, as the hybrid has')
    ap.add_argument('--name')
    ap.add_argument('--compare', action='store_true')
    ap.add_argument('--finish', action='store_true')
    ap.add_argument('--trainer', default=os.path.join(ROOT, '..', 'milli-rl', 'src', 'mjlab_microduck', 'robot'))
    a = ap.parse_args()
    height = a.height_cm / 100
    base = recipes()
    servos = list(CATALOG) if a.compare or not a.servo else [a.servo]
    rows = []
    for servo in servos:
        if servo not in CATALOG:
            sys.exit(f'{servo} is not in the catalog: {list(CATALOG)}. Add its numbers to spec/actuators.json and KP in tools/make_rl_model.py, and a line to CATALOG')
        name = a.name if a.name and len(servos) == 1 else re.sub(r'[^a-z0-9]', '', f'd{a.height_cm:g}{servo.split("_")[0].lower()}{servo.split("_")[-1].lower()}')
        K, m, too_small = fit(name, servo, height, base[CATALOG[servo]], a.head_servo)
        fails = checks(name)
        trainer_models(name, a.trainer)
        mg, stall = margin(name, a.trainer)
        rows.append(dict(name=name, servo=servo, K=K, too_small=too_small, fails=fails, margin=mg, stall=stall, **m))
        print(f"{servo:14s} {name:24s} K {K:.2f}: {m['height'] * 100:.1f} cm tall, {m['width'] * 100:.1f} cm wide, {m['mass']:.2f} kg; head pitch stall {stall:.2f} N m, roll margin {mg:.2f}; {len(fails)} checks failing"
              + (f"; SMALLEST this layout reaches, {(m['height'] - height) * 100:.1f} cm over the target" if too_small else ''), flush=True)
        for ln in fails[:12]:
            print('    ' + ln[:170], flush=True)
    print(f"roll margin: stall over the torque to hold the tucked body on the head. Below {ROLL_MARGIN} expect no roll without a stronger neck (a static estimate: test with tools/flip_search.py). Stall figures: spec/actuators.json")
    if a.finish:
        for r in rows:
            if r['fails'] or r['too_small']:
                print(f"{r['name']}: not finished: {len(r['fails'])} checks failing" + (', and the height not reached' if r['too_small'] else ''))
                continue
            lie, sit, stand = clearances(r['name'], a.trainer)
            json.dump(dict(height=r['height'] / 0.264, stand=stand - 0.001, prone=[lie + 0.002, lie + 0.025], sit=[sit + 0.002, sit + 0.02]), open(os.path.join(a.trainer, f"milli_{r['name']}_contact", 'trainer.json'), 'w'), indent=1)
            print(f"{r['name']}: trainer tasks: Mjlab-Velocity-Flat-Milli-{r['name'].capitalize()}, Mjlab-StandUp-Flat-Milli-{r['name'].capitalize()}, Mjlab-Roulade-Flat-Milli-{r['name'].capitalize()}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
