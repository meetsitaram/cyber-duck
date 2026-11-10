"""docs/big/wiring.png: the big duck's power and CAN, drawn from its actuators. MILLI_RECIPE isn't read: python -I tools/big_wiring.py

Stops if the spec's actuators stop matching the buses in docs/big/WIRING.md."""
import collections
import json
import os

from PIL import Image, ImageDraw, ImageFont

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
spec = json.load(open(os.path.join(ROOT, 'spec', 'robot_big.json'), encoding='utf-8'))
have = collections.Counter(g['name'].split()[-1] for l in spec['links'].values() for g in l['geoms'] if g['role'] == 'actuator')
LEG = ['hip yaw', 'hip roll F', 'hip roll B', 'hip pitch I', 'hip pitch O', 'knee I', 'knee O', 'ankle']
HEAD = [('neck L', 'RS04'), ('neck R', 'RS04'), ('head pitch', 'RS04'), ('head yaw', 'RS03'), ('head roll', 'RS03'), ('beak', 'RS00')]
assert have == {'RS04': 2 * len(LEG) + 3, 'RS03': 2, 'RS00': 1}, have

W, H = 1960, 1110
RED, BLUE, GREY, INK, PALE, AMBER = '#C4332A', '#1F5FBF', '#6B6F76', '#20242A', '#F2F3F4', '#B86E00'
im = Image.new('RGB', (W, H), 'white')
d = ImageDraw.Draw(im)


def font(s, bold=False):
    for f in (('arialbd.ttf' if bold else 'arial.ttf'), 'DejaVuSans.ttf'):
        try:
            return ImageFont.truetype(f, s)
        except OSError:
            pass
    return ImageFont.load_default()


def box(x, y, w, h, title, sub=(), fill=PALE, edge=INK):
    d.rounded_rectangle((x, y, x + w, y + h), 8, fill=fill, outline=edge, width=2)
    d.text((x + 10, y + 7), title, fill=INK, font=font(19, True))
    for i, t in enumerate(sub):
        d.text((x + 10, y + 33 + i * 21), t, fill=INK, font=font(16))


def line(pts, col, w=4):
    d.line(pts, fill=col, width=w, joint='curve')


def label(x, y, t, col=INK, s=16, b=False):
    d.text((x, y), t, fill=col, font=font(s, b))


label(30, 18, 'Big duck: power and CAN', s=30, b=True)
label(30, 58, 'NOT SAFE TO BUILD. A paper design: nothing wired, powered or measured. No brakes: cutting power drops about 100 kg.', RED, 20, True)
line([(1330, 30), (1385, 30)], RED, 5)
label(1395, 19, '48 V bus, about 54.6 V full', s=18)
line([(1330, 62), (1385, 62)], BLUE, 3)
label(1395, 51, 'CAN, 1 Mbps, twisted pair', s=18)
line([(1330, 94), (1385, 94)], AMBER, 3)
label(1395, 83, '24 V or 5 V, or signal', s=18)

# power side
box(30, 150, 270, 110, 'Battery: candidate only', ['Dan-Tech 13S5P, 1200 Wh, 100 A', 'BMS and case: not chosen'], '#FBEDEC')
box(30, 300, 270, 66, 'Main fuse, disconnect', ['not chosen'], '#FBEDEC')
box(30, 420, 270, 90, 'PRE/DIS_R resistor', ['10 ohm, 200 W; 2-way terminal', 'needs a metal plate to cool'], '#FBEDEC')
box(30, 545, 270, 90, 'Emergency stop', ['EM_NO terminal, normally open', 'cuts branches: the robot falls'], '#FBEDEC')
box(380, 150, 330, 500, 'Power board, RobStride', ['90 x 110 mm, 20-65 V in', '', 'BAT: not named, looks like XT60', 'branches: 4 x XT30', '20 A rated, 40 A peak each', '80 A total for 30 ms', '', '24 V 4 A, 12 V 2 A, 5 V 2 A', 'CAN, USB-C debug, FAN', '', 'SEQ switches at 11: nothing', 'starts by itself', 'Discharge parameter 0x8003 = 54,', 'not its default 58'])
line([(165, 260), (165, 300)], RED)
line([(300, 333), (340, 333), (340, 200), (380, 200)], RED)
line([(300, 465), (380, 465)], GREY, 3)
line([(300, 590), (380, 590)], GREY, 3)

# computer
box(380, 730, 330, 90, 'Regulator: not chosen', ['24 V to 5 V, 5 A'], '#FBEDEC')
line([(500, 650), (500, 730)], AMBER)
label(510, 680, '24 V', s=16)
box(380, 880, 330, 96, 'Raspberry Pi 5', ['policy, in the body', 'I2C to BNO085 IMU; USB camera optional'])
line([(500, 820), (500, 880)], AMBER)
label(510, 842, 'USB-C', s=16)
box(30, 880, 270, 96, 'IMU: BNO085', ['4 wires, body'])
line([(300, 928), (380, 928)], AMBER, 3)
box(770, 880, 300, 96, 'USB to CAN, 3 of 5 channels', ['RobStride hub: not confirmed', 'Terminator at this end: 120 ohm'])
line([(710, 928), (770, 928)], AMBER, 3)
label(722, 900, 'USB', s=15)


def run(y, title, names, ids, branch, can, tag, x0=1140):
    """One bus: its distribution, then a row of actuators."""
    box(770, y, 300, 86, f'Distribution: not designed', [f'{branch} XT30 in, {len(names)} XT30 out', '12 AWG, rated 40 A at 60 V'], '#FBEDEC')
    label(x0, y - 26, title, s=19, b=True)
    w = 86
    for i, n in enumerate(names):
        nm, a = n if isinstance(n, tuple) else (n, 'RS04')
        x = x0 + i * (w + 4)
        d.rounded_rectangle((x, y, x + w, y + 86), 6, fill='#E8ECF2', outline=INK, width=2)
        for k, t in enumerate(nm.split(' ', 1)):
            d.text((x + 6, y + 5 + k * 18), t, fill=INK, font=font(15, True))
        d.text((x + 6, y + 44), a, fill=INK, font=font(14))
        d.text((x + 6, y + 62), f'ID {ids + i}', fill=INK, font=font(14))
    r = x0 + (len(names) - 1) * (w + 4) + w
    line([(1070, y + 24), (x0 - 12, y + 24)], RED)
    line([(1070, y + 100), (r, y + 100)], BLUE, 3)
    for i in range(len(names)):
        x = x0 + i * (w + 4) + w // 2
        line([(x, y + 86), (x, y + 100)], BLUE, 3)
    d.rectangle((r + 6, y + 91, r + 20, y + 109), fill='white', outline=BLUE, width=2)
    label(r + 2, y + 112, '120 ohm', BLUE, 13)
    return y + 100


y1 = run(190, 'can0: left leg, branch LL', LEG, 11, 'LL', 'can0', 'left leg')
y2 = run(390, 'can1: right leg, branch LR', LEG, 21, 'LR', 'can1', 'right leg')
y3 = run(590, 'can2: neck and head, branch AL', HEAD, 31, 'AL', 'can2', 'neck, head')
for y, c in ((230, RED), (430, RED), (630, RED)):
    line([(710, y), (770, y)], c)
for i, yb in enumerate((y1, y2, y3)):
    x = 1090 + i * 16
    line([(1070, yb), (x, yb), (x, 860), (1040 - i * 80, 860), (1040 - i * 80, 880)], BLUE, 3)
label(1140, 745, 'At each actuator: XT30UW-F to XT30U-M for power, GH1.25 2-pin for CAN.', s=17)
label(1140, 768, 'The RS00 is one plug: XT30(2+2)-F.G.B.', s=17)
label(1140, 800, 'Harness 310 mm, CAN pigtail 40 mm more: a 350 mm stub, past the usual', s=17)
label(1140, 823, '300 mm at 1 Mbps. Each has one CAN plug, so the bus taps, not chains.', s=17)
label(1140, 855, 'Measure H to L with everything off: about 60 ohm per bus.', s=17)
label(1140, 887, 'Branch AR: spare.', s=17)
label(30, 1020, 'Connector status and all that is unknown: docs/big/WIRING.md, docs/big/BOM.md. Boxes in pink are not chosen or not designed.', s=20)
label(30, 1055, 'Limits are from RobStride\'s manuals. No current here has been measured: a tripped branch stops a leg until restart, and the robot falls.', RED, 20, True)

out = os.path.join(ROOT, 'docs', 'big', 'wiring.png')
os.makedirs(os.path.dirname(out), exist_ok=True)
im.save(out, optimize=True)
print(out)
