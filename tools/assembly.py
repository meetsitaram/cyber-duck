"""How milli goes together: every part and screw, in the order a person can do it, and the checks on that order.
(CDX-3R's lessons: "every check looked only at the finished assembly"; a collar that could not go on, screw heads sealed
inside a grip.) Nothing here is drawn: the parts are spec/robot.py's geoms, grouped into what you print or buy.

  python -I tools/assembly.py              the checks; report in reports/assembly.md, exit 1 on any FAIL
  python -I tools/assembly.py --bad CASE   break the order on purpose and show the check FAIL (seal thread order path hold apart window)

The checks, at the zero pose:
  ALL     every rigid part and every screw is fitted exactly once
  HOLD    every printed part is really held: by screws not all on one axis (it would turn about them), or named as glued or as a pivot
  APART   no two parts of the robot share space: not even two parts of one link, except what is meant to seat or thread
  PATH    each part slides in along its path without cutting into parts already on
  THREAD  a screw's tip half bites 2 mm into something it is not clamping: a servo, a horn or a printed part
  HELD    under its head, the screw clamps a printed part
  ACCESS  a key reaches the head: 15 mm straight out from it clear of everything already on (CDX-3R's KEY_REACH)
"""
import argparse
import os
import re
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import checks as C  # noqa: E402
import geom as G  # noqa: E402

# Printed parts that are glued or that turn on their screws, on purpose
GLUED = {'jaw soft insert', 'mouth roof', 'face', 'eye ring', 'eye iris'}
PIVOTS = {'jaw'}
KEY_REACH, KEY_R = 0.015, 0.0012                   # a hex key's short arm, and its radius
MIN_BITE = 0.002
TRAVEL = 0.045                                     # how far a part is checked along its path

X, Y, Z = (1, 0, 0), (0, 1, 0), (0, 0, 1)
NEG = lambda v: tuple(-a for a in v)


class Past(tuple):
    """A way of travel with parts held out of the path for this step: Past(Z, ['jaw']), the jaw hanging on its rod."""

    def __new__(cls, way, aside):
        o = super().__new__(cls, way)
        o.aside = aside
        return o


class Short(tuple):
    """A way of travel checked for less than TRAVEL: a small part that a hand brings in from the side, and only its last stretch is along its axis."""

    def __new__(cls, way, travel):
        o = super().__new__(cls, way)
        o.travel = travel
        return o


# a hobby servo's tray also has the rims its ears screw down on, and a post under the near rim; its lid a foot down onto the far ears (spec/robot.py flange_cradle)
EARS = ('cradle rim far', 'cradle rim near', 'cradle near post')


def tray(n, sides='+-', rear=True):
    return ([f'{n} cradle rear face'] if rear else []) + [f'{n} cradle end +', f'{n} cradle end -'] + \
        [f'{n} cradle side {s}' for s in sides] + [f'{n} {e}' for e in EARS]


def lid(n, seat='seat floor'):
    return [f'{n} cradle horn face', f'{n} horn {seat}', f'{n} cradle horn face foot', f'{n} cradle horn face cap']


def hub(n):
    """A hobby servo's far side on a fork: the printed hub through the fork's plate into a bearing in the tray's floor."""
    return (f'{n} rear hub', 'P', [f'{n} rear hub shoulder', f'{n} rear hub boss', f'{n} rear hub plug', f'{n} rear hub collar'])


AV2 = RF2 = G.RECIPE == 'dynamixel'                      # (the ankle of 2026-11-09: spec/robot.py's ankle_v2)
BODY_PI = G.RECIPE == 'hobby'                      # the hobby duck's Pi is in its body: it goes in with the torso, not the head


def races(n, side='horn'):
    return [f'{n} {side} bearing outer race', f'{n} {side} bearing inner race']


# (name, kind, geoms): printed (P), bought (B). Written for the left leg; the right is its mirror.
def leg_parts(s):
    y, r, p, k, a = (f'{s} hip yaw', f'{s} hip roll', f'{s} hip pitch', f'{s} knee', f'{s} ankle pitch')
    return [
        (f'{s} hip yaw servo', 'B', [f'{s}_hip_yaw actuator (buy) STS3215_C046', f'{y} cable plugs']),
        (f'{s} hip yaw lid', 'P', lid(y)),
        (f'{s} hip yaw bearing', 'B', races(y)),
        (f'{s} hip yaw horn', 'B', [f'{s}_hip_yaw flange']),
        (f'{s} yaw bracket', 'P', [f'{s} hip yaw plate', f'{y} horn boss shoulder'] + ([f'{s} hip yaw near tab {f}{q}' for f in 'hr' for q in (0, 1)] if RF2 else tray(r, '+' if s == 'L' else '-'))),
        (f'{s} hip roll servo', 'B', [f'{s}_hip_roll actuator (buy) STS3215_C044', f'{r} cable plugs']),
        *([(f'{s} hip roll idler', 'B', [f'{r} rear horn'])] if RF2 else [(f'{s} hip roll lid', 'P', lid(r)), (f'{s} hip roll bearing', 'B', races(r))]),
        (f'{s} hip roll horn', 'B', [f'{s}_hip_roll flange']),
        (f'{s} roll bracket', 'P', [f'{s} hip roll front', f'{s} hip roll side', f'{s} hip roll arm', f'{s} roll gusset 0', f'{s} roll gusset 1', *([f'{s} hip roll back arm', f'{r} fork horn plate', f'{r} fork rear plate', f'{r} horn seat boss', f'{r} rear seat boss'] if RF2 else [f'{r} horn boss shoulder']),
                                    f'{p} horn boss shoulder']),
        (f'{s} thigh frame', 'P', [f'{s} thigh plate', f'{s} thigh post hip', f'{s} thigh post knee'] + tray(p) + tray(k)),
        (f'{s} hip pitch servo', 'B', [f'{s}_hip_pitch actuator (buy) STS3215_C044', f'{p} cable plugs']),
        (f'{s} knee servo', 'B', [f'{s}_knee actuator (buy) STS3215_C044', f'{k} cable plugs']),
        (f'{s} hip pitch lid', 'P', lid(p)),
        (f'{s} knee lid', 'P', lid(k)),
        (f'{s} hip pitch bearing', 'B', races(p)),
        (f'{s} knee bearing', 'B', races(k)),
        (f'{s} hip pitch horn', 'B', [f'{s}_hip_pitch flange']),
        (f'{s} knee horn', 'B', [f'{s}_knee flange']),
        (f'{s} thigh cover', 'P', [f'{s} thigh cover']),
        (f'{s} shin', 'P', [f'{s} shin plate', f'{s} shin rib', f'{s} shin gusset', f'{s} shin collar 0', f'{s} shin collar 1', f'{s} shin cap', *([f'{s} shin arm', f'{s} shin sleeve rear'] if AV2 else [f'{s} ankle stub plate', f'{s} ankle rail 0', f'{s} ankle rail 1', f'{s} ankle stub']),
                            f'{k} horn boss shoulder', f'{a} cradle rear face', f'{a} cradle end +', f'{a} cradle end -'] + [f'{a} {e}' for e in EARS]),
        (f'{s} ankle servo', 'B', [f'{s}_ankle_pitch actuator (buy) STS3215_C044', *([] if AV2 else [f'{s} ankle rear boss']), f'{a} cable plugs']),
        (f'{s} ankle lid', 'P', [f'{a} cradle horn face']),
        (f'{s} ankle horn', 'B', [f'{s}_ankle_pitch flange']),
        *([(f'{s} ankle idler', 'B', [f'{s} ankle rear boss'])] if AV2 else [(f'{s} ankle bearing inner', 'B', [f'{s} ankle bearing inner race', f'{s} ankle bearing outer race'])]),
        (f'{s} shoe', 'P', [f'{s} shoe', f'{s} ankle ear out', *([f'{s} ankle ear in'] if AV2 else [])]),
        (f'{s} sole', 'P', [f'{s} sole']),
        (f'{s} foot switch', 'B', [f'{s} foot switch']),
        *([] if AV2 else [(f'{s} inner ear', 'P', [f'{s} ankle ear in', f'{s} ankle ear hub'])]),
    ]


PARTS = leg_parts('L') + leg_parts('R') + [
    ('chassis', 'P', ['top plate', 'neck wall', 'neck wall tab', 'L neck side wall', 'L neck side wall top', 'R neck side wall', 'R neck side wall top', 'neck mount wings', 'neck top bracket horn', 'neck top bracket rear', 'neck mount', 'neck bracket', 'neck idler bracket', 'battery strap', 'pack hanger 0', 'pack hanger 1',
                      'neck idler step', 'Pi post 0', 'Pi post 1', 'Pi post 2', 'Pi post 3', 'neck pitch horn seat boss', 'neck pitch rear seat boss', 'neck pitch fork horn plate', 'neck pitch fork rear plate', 'neck pitch fork post', 'body post 0', 'body post 1', 'L regulator arm 0', 'L regulator arm 1', 'R regulator arm 0', 'R regulator arm 1', 'L regulator drop 0', 'L regulator drop 1', 'R regulator drop 0', 'R regulator drop 1']
     + tray('L hip yaw') + tray('R hip yaw')),
    ('battery', 'B', ['battery 2S 850 mAh'] + ([] if BODY_PI else ['battery XT30', 'battery balance plug'])),
    ('battery plugs', 'B', ['battery XT30', 'battery balance plug'] if BODY_PI else []),   # (on their leads: pulled out through the port once the taller shell is on)
    ('L 5 V regulator', 'B', ['L 5 V regulator']),
    ('R 5 V regulator', 'B', ['R 5 V regulator']),
    ('power switch', 'B', ['power switch', 'power switch neck', 'power switch leads', 'power switch bezel', 'power switch rocker']),
    ('head 5 V regulator', 'B', ['head 5 V regulator', 'head regulator plug', 'head regulator wires']),
    ('IMU', 'B', ['IMU']),
    ('body shell', 'P', ['body']),
    ('neck servo', 'B', ['neck_pitch actuator (buy) XC330_M288', 'neck_pitch flange', 'neck pitch rear horn', 'neck pitch cable plugs']),
    ('neck column', 'P', ['neck front plate', 'neck back plate', 'neck pitch cradle rear face', 'neck pitch cradle end +',
                          'neck pitch cradle end -', 'neck pitch cradle side +', 'neck pitch cradle side -',
                          'head pitch cradle rear face', 'head pitch cradle end +', 'head pitch cradle end -',
                          'head pitch cradle side +', 'head pitch cradle side -', 'neck pitch rear seat floor', 'head pitch rear seat floor']
     + [f'{n} {e}' for n in ('neck pitch', 'head pitch') for e in EARS]),
    ('neck lid', 'P', ['neck pitch cradle horn face', 'neck pitch cradle horn face foot', 'neck pitch cradle horn face cap']),
    ('neck pitch rear bearing', 'B', races('neck pitch', 'rear')),
    ('head pitch rear bearing', 'B', races('head pitch', 'rear')),
    hub('neck pitch'), hub('head pitch'), hub('head roll'),
    ('L 6 V BEC', 'B', ['L 6 V BEC']),
    ('R 6 V BEC', 'B', ['R 6 V BEC']),
    ('Pi 5 V converter', 'B', ['Pi 5 V converter']),
    ('PWM bonnet', 'B', ['PWM bonnet', 'PWM bonnet header', 'PWM bonnet plugs']),
    ('head pitch servo', 'B', ['head_pitch actuator (buy) XL330_M288', 'head_pitch flange', 'head pitch rear horn', 'head pitch cable plugs']),
    ('head pitch lid', 'P', ['head pitch cradle horn face', 'head pitch cradle horn face foot', 'head pitch cradle horn face cap']),
    ('head fork', 'P', ['head pitch horn seat boss', 'head pitch rear seat boss', 'head pitch plate', 'head pitch idler plate', 'head pitch bridge', 'head pitch fork horn plate',
                        'head pitch fork rear plate'] + tray('head yaw')),
    ('head yaw servo', 'B', ['head_yaw actuator (buy) XL330_M288', 'head yaw cable plugs']),
    ('head yaw sheet', 'P', lid('head yaw', 'seat wall') + ['head yaw horn seat floor']),
    ('head yaw bearing', 'B', races('head yaw')),
    ('head yaw horn', 'B', ['head_yaw flange']),
    ('head yaw bracket', 'P', ['head yaw plate', 'head yaw horn boss shoulder', 'head roll cradle end +', 'head roll cradle end -',
                               'head roll cradle side +', 'head roll cradle side -', 'head roll cradle rear face',
                               'head roll rear seat floor'] + [f'head roll {e}' for e in EARS]),
    ('head roll servo', 'B', ['head_roll actuator (buy) XL330_M288', 'head roll cable plugs']),
    ('head roll rear bearing', 'B', races('head roll', 'rear')),
    ('head roll rear horn', 'B', ['head roll rear horn']),
    ('head roll lid', 'P', lid('head roll')),
    ('head roll bearing', 'B', races('head roll')),
    ('head roll horn', 'B', ['head_roll flange']),
    # (loose, not on the head frame: it drops down between them, so nothing of it can stand in a bearing's bore)
    ('head roll spacer', 'P', [] if BODY_PI else ['head roll horn seat boss']),   # (the hobby duck's is 0.4 mm, too thin to print alone: part of the frame)
    ('head roll rear spacer', 'P', [] if G.RECIPE == 'feetech' else ['head roll rear seat boss']),   # (no rear bearing on the all-Feetech duck, so nothing for it to pass: it's part of the frame)
    ('head frame', 'P', ['head frame', 'head top plate', 'head top plate wide', 'L head rail', 'L head rail front', 'R head rail',
                         'L head hinge boss', 'R head hinge boss', 'Pi mount', 'Pi mount 2', 'Pi board tab', 'bus board post 2', 'bus board post 3', 'Pi plate', 'Pi boss 0', 'Pi boss 1', 'Pi boss 2', 'Pi boss 3', 'bus board rail 0', 'bus board rail 1', 'bus board post 0', 'bus board post 1', 'rear board plate', 'rear board plate low', 'rear board plate low 2', 'rear board foot', 'rear board foot 2', 'rear board boss 0', 'rear board boss 1', 'rear board boss 2', 'rear board boss 3', 'lower shell', 'lower shell post', 'head base', 'head base wall', 'Pi base tab', 'L base riser', 'L base riser top', 'R base riser', 'lower shell beam',
                         *(['head roll rear seat boss'] if G.RECIPE == 'feetech' else []), 'head roll horn boss shoulder', 'head roll rear boss shoulder', 'head roll fork horn plate',
                         'head roll fork rear plate', 'head roll fork post', 'beak cradle end plain', 'head strip deep', 'L base riser back', 'beak cradle rear face', 'beak cradle end +',
                         'beak cradle end -', 'beak cradle side +'] + [f'beak {e}' for e in EARS]),
    ('beak servo', 'B', ['beak actuator (buy) XL330_M288', 'beak flange', 'beak cable plugs']),
    ('beak lid', 'P', ['beak cradle horn face']),
    ('beak crank', 'P', ['beak crank', 'beak crank pin']),
    ('beak rod', 'P', ['beak rod']),
    ('Pi Zero 2 W', 'B', ['Pi Zero 2 W', 'Pi USB plug', 'Pi PWR plug', 'Pi camera ribbon', 'Pi microSD', 'Pi GPIO wires']),
    ('camera', 'B', ['camera']),
    ('servo bus board', 'B', ['servo bus board']),
    ('Waveshare bus board', 'B', ['Waveshare bus board', 'bus board cable plugs servo', 'bus board cable plugs power', 'bus board cable plugs usb', 'bus board cable plugs uart']),
    ('bus board standoffs', 'B', [f'bus board standoff {k}' for k in range(4)]),
    ('U2D2', 'B', ['U2D2']),
    ('leg bus board', 'B', ['leg bus board', 'leg bus board plugs servo', 'leg bus board plugs uart', 'leg bus board plugs power']),
    ('leg bus splice', 'B', ['leg bus splice']),
    ('hood', 'P', ['head', 'upper bill', 'hood rear tab 0', 'hood rear tab 1', 'L hood step', 'R hood step']),
    ('jaw', 'P', ['beak horn seat boss', 'jaw', 'L hinge tab', 'R hinge tab', 'L tab foot', 'R tab foot', 'jaw lever', 'L tab hub', 'R tab hub']),
    ('jaw soft insert', 'P', ['jaw soft']),
    ('mouth roof', 'P', ['mouth roof']),
    ('face', 'P', ['face', 'sensor']),
    ('eye ring', 'P', ['eye ring']),
    ('eye iris', 'P', ['eye lens']),
]


def mir(v, s):
    return (v[0], v[1] if s == 'L' else -v[1], v[2])


def sections():
    """The build, in order. on: sections already together when this one starts (the rest of the robot is elsewhere:
    a bench job). Each step: (caption, [(part or @section, the way it travels into place)], [screw name patterns])."""
    out = []
    for s, nm in (('L', 'left'), ('R', 'right')):
        y, r, p, k, a = (f'{s} hip yaw', f'{s} hip roll', f'{s} hip pitch', f'{s} knee', f'{s} ankle pitch')
        OUT, IN = mir(Y, s), mir(NEG(Y), s)
        out += [dict(key=f'thigh_{s}', title=f'The {nm} thigh', on=[], steps=[
            ('The thigh frame', [(f'{s} thigh frame', None)], []),
            ('Plug in the cables, then drop in both servos', [(f'{s} hip pitch servo', OUT), (f'{s} knee servo', OUT)], []),
            ('The screws from the outside', [], [rf'^{p} cradle screw rear', rf'^{k} cradle screw rear']),
            ('The lids', [(f'{s} hip pitch lid', OUT), (f'{s} knee lid', OUT)], []),
            ('The lid screws', [], [rf'^{p} cradle screw (horn|near)', rf'^{k} cradle screw (horn|near)']),
            ('Press in the bearings', [(f'{s} hip pitch bearing', OUT), (f'{s} knee bearing', OUT)], []),
            ('Horns on, servos at mid-range', [(f'{s} hip pitch horn', OUT), (f'{s} knee horn', OUT)], []),
            ('The cover, 2 screws', [(f'{s} thigh cover', IN)], [rf'^{s} thigh cover screw']),
        ]), dict(key=f'shin_{s}', title=f'The {nm} shin', on=[], steps=[
            ('The shin', [(f'{s} shin', None)], []),
            ('Plug in the cable, slide in the ankle servo', [(f'{s} ankle servo', Z if AV2 else IN)], []),
            ('Screws: 2 through the far ears, 2 through the near ears' if BODY_PI else '4 screws at the far end, 2 on each face' if AV2 else 'Screws: 4 flat-head into the rear horn, 2 at the far end', [], ([] if BODY_PI or AV2 else [rf'^{s} ankle boss screw']) + ([rf'^{s} shin arm screw'] if AV2 else []) + [rf'^{a} cradle screw rear']),
            ('The lid, 2 screws', [(f'{s} ankle lid', IN)], [rf'^{a} cradle screw horn']),
            *([] if AV2 else [('Press the small bearing onto the stub', [(f'{s} ankle bearing inner', OUT)], [])]),
            ('Horn on, servo at mid-range', [(f'{s} ankle horn', IN)], []),
        ]), dict(key=f'shoe_{s}', title=f'The {nm} shoe', on=[], steps=[
            ('The shoe', [(f'{s} shoe', None)], []),
            *([('Press in the foot switch, its wires soldered first', [(f'{s} foot switch', Z)], [])] if G.RECIPE not in ('dynamixel', 'hd1910') else []),
            ('The TPU sole, 2 screws from below', [(f'{s} sole', Z)], [rf'^{s} shoe bottom screw']),
        ]), dict(key=f'leg_{s}', title=f'The {nm} leg', on=[f'thigh_{s}'], steps=[
            ('The shin onto the knee horn', [(f'@shin_{s}', OUT)], []),
            ('4 screws from the inside', [], [rf'^{k} fork screw']),
            ('Lift the shoe onto the ankle servo', [(f'@shoe_{s}', Z)], []),
            ('4 screws through the outer ear', [], [rf'^{a} fork screw']),
            ('4 screws through the inner ear, into the idler', [(f'{s} ankle idler', None)], [rf'^{s} ankle ear screw']) if AV2 else ('The inner ear onto the stub, 2 screws', [(f'{s} inner ear', OUT)], [rf'^{s} ankle ear screw']),
            ('The roll bracket onto the hip horn, 4 screws', [(f'{s} roll bracket', OUT)], [rf'^{p} fork screw']),
        ])]
    out += [dict(key='body', title='The chassis and hips', on=[], steps=[
        ('The chassis, upside down', [('chassis', None)], []),
        ('Plug in the cables, push in the hip yaw servos', [('L hip yaw servo', Z), ('R hip yaw servo', Z)], []),
        ('2 screws each, from the top', [], [r'^[LR] hip yaw cradle screw rear']),
        ('The lids and their screws', [('L hip yaw lid', Z), ('R hip yaw lid', Z)], [r'^[LR] hip yaw cradle screw (horn|near)']),
        ('Press in the bearings', [('L hip yaw bearing', Z), ('R hip yaw bearing', Z)], []),
        ('Horns on, servos at mid-range', [('L hip yaw horn', Z), ('R hip yaw horn', Z)], []),
        ('The yaw brackets, 4 screws each, before the roll servos', [('L yaw bracket', Z), ('R yaw bracket', Z)], [r'^[LR] hip yaw fork screw']),
        *([('Plug in the cables, slide the hip roll servos up into the yaw brackets, horns on', [('L hip roll servo', Z), ('L hip roll horn', Z), ('R hip roll servo', Z), ('R hip roll horn', Z)], []),
           ('4 screws each, 2 on each face, at the top', [], [r'^[LR] hip roll cradle screw near']),
           ('Idlers on, servos at mid-range', [('L hip roll idler', X), ('R hip roll idler', X)], [])] if RF2 else [
        ('Plug in the cables, slide in the hip roll servos', [('L hip roll servo', NEG(X)), ('R hip roll servo', NEG(X))], []),
        ('2 screws each, from behind', [], [r'^[LR] hip roll cradle screw rear']),
        ('The lids and their screws', [('L hip roll lid', NEG(X)), ('R hip roll lid', NEG(X))], [r'^[LR] hip roll cradle screw (horn|near)']),
        ('Press in the bearings', [('L hip roll bearing', NEG(X)), ('R hip roll bearing', NEG(X))], []),
        ('Horns on, servos at mid-range', [('L hip roll horn', NEG(X)), ('R hip roll horn', NEG(X))], [])]),
        ('A 6 V BEC up into each side pocket, tied to its 2 arms, before the legs', [('L 6 V BEC', Z), ('R 6 V BEC', Z)], []),
    ]), dict(key='legs', title='Legs onto the hips', on=['body'], steps=[
        ('The left leg onto the roll horn' + (' and idler, its fork round the servo from inside' if RF2 else ''), [('@leg_L', NEG(Y) if RF2 else NEG(X))], []),
        ('4 screws from the front' + (', 4 from behind' if RF2 else ''), [], [r'^L hip roll fork screw']),
        ('The right leg', [('@leg_R', Y if RF2 else NEG(X))], []),
        ('4 screws from the front' + (', 4 from behind' if RF2 else ''), [], [r'^R hip roll fork screw']),
    ]), dict(key='column', title='The neck column', on=[], steps=[
        ('The neck column', [('neck column', None)], []),
        ('Neck servo in, horns on', [('neck servo', NEG(Y))], [r'^neck pitch cradle screw rear']),
        ('Its lid, 2 screws', [('neck lid', NEG(Y))], [r'^neck pitch cradle screw horn']),
        ('Head pitch servo in, horns on', [('head pitch servo', Y)], [r'^head pitch cradle screw rear']),
        ('Its lid, 2 screws', [('head pitch lid', Y)], [r'^head pitch cradle screw horn']),
        ('Press the rear bearings into the trays\' floors', [('neck pitch rear bearing', Y), ('head pitch rear bearing', NEG(Y))], []),
    ]), dict(key='torso', title='Neck and body shell', on=['legs'], steps=[
        ('Battery into its slot from behind, onto its tray; a hook-and-loop strap round both', [('battery', X)], []),
        *([('The 5 V regulator into its side pocket from outside, tied to its 2 arms', [('L 5 V regulator', NEG(Y))], []),
           ('The legs\' bus splice on the top plate, foam tape, both hip yaw leads plugged in', [('leg bus splice', NEG(Z))], [])] if G.RECIPE == 'hybrid' else [('A 5 V regulator into each side pocket', [('L 5 V regulator', Z), ('R 5 V regulator', Z)], [])]),
        ('The IMU on the top plate, double-sided tape', [('IMU', NEG(Z))], []),
        ('The Pi\'s 5 V converter on the plate beside it, foam tape', [('Pi 5 V converter', NEG(Z))], []),
        *([('The Pi onto its 4 posts, microSD in, camera ribbon on, 4 screws', [('Pi Zero 2 W', NEG(Z))], [r'^Pi screw']),
           ('The PWM bonnet onto the Pi\'s GPIO header, then each servo\'s lead into its channel', [('PWM bonnet', NEG(Z))], [])] if BODY_PI else []),
        *([('The body shell, 2 screws, before the neck: the neck column is as wide as the body', [('body shell', NEG(Z))], [r'^body screw']),
           ('The battery\'s two plugs out through the port in its back', [('battery plugs', Short(NEG(X), 0.004))], [])] if BODY_PI else []),
        ('The column down into the chassis fork', [('@column', NEG(Z))], []),
        ('4 screws on each horn', [], [r'^neck pitch fork screw']),
        ('The rear hub through the fork into its bearing, 2 screws', [('neck pitch rear hub', Y)], [r'^neck pitch hub screw']),
        *([] if BODY_PI else [('The body shell, 2 screws', [('body shell', NEG(Z))], [r'^body screw'])]),
        *([("The rocker switch: its 2 leads spliced into the battery's red lead before the shell goes on, then pushed out through the hole in the back. Once the shell is on, push the rocker in from outside till it clicks. The XT30 stays plugged", [('power switch', X)], [])] if G.RECIPE == 'dynamixel' else []),
    ]), dict(key='headbench', title='The head yaw and roll', on=[], steps=[
        ('The head fork', [('head fork', None)], []),
        ('Head yaw servo in, 2 screws from under the bridge', [('head yaw servo', NEG(Z))], [r'^head yaw cradle screw rear']),
        ('The thin sheet, 2 screws', [('head yaw sheet', NEG(Z))], [r'^head yaw cradle screw (horn|near)']),
        ('Bearing and horn', [('head yaw bearing', NEG(Z)), ('head yaw horn', NEG(Z))], []),
        ('The yaw bracket, 4 screws, before its roll servo', [('head yaw bracket', NEG(Z))], [r'^head yaw fork screw']),
        ('Head roll servo in from the front', [('head roll servo', NEG(X))], [r'^head roll cradle screw rear']),
        ('The rear bearing and rear horn, from behind', [('head roll rear bearing', X), ('head roll rear horn', X), ('head roll rear spacer', X)], []),
        ('The lid, 2 screws', [('head roll lid', NEG(X))], [r'^head roll cradle screw (horn|near)']),
        ('Bearing, horn and spacer', [('head roll bearing', NEG(X)), ('head roll horn', NEG(X)), ('head roll spacer', NEG(X))], []),
    ]), dict(key='framebench', title='The head frame and the beak servo', on=[], steps=[
        ('The head frame', [('head frame', None)], []),
        ('The bus board\'s brass standoffs into their hex sockets, a screw from the front into each', [('bus board standoffs', Short(X, 0.018) if G.RECIPE == 'feetech' else X)], [r'^bus board rear screw']),
        ('Beak servo in from the side, its horn out, servo at mid-range, 4 screws from inside (the roll servo blocks them later)', [('beak servo', NEG(Y))], [r'^beak cradle screw rear']),
    ]), dict(key='jawbench', title='The jaw', on=[], steps=[
        ('The jaw', [('jaw', None)], []),
        ('Glue in its soft insert', [('jaw soft insert', NEG(Z))], []),
    ]), dict(key='head', title='The head', on=['torso'], steps=[
        ('The head fork down onto the head pitch servo', [('@headbench', NEG(Z))], []),
        ('4 screws on each horn', [], [r'^head pitch fork screw']),
        ('The rear hub through the fork into its bearing, 2 screws', [('head pitch rear hub', NEG(Y))], [r'^head pitch hub screw']),
        ('The head frame, with its beak servo, down over the roll servo' + (', the wires from the neck up through its opening' if G.RECIPE == 'hybrid' else ''), [('@framebench', NEG(Z))], []),
        ('4 screws on each horn', [], [r'^head roll fork screw']),
        ('The rear hub through the fork into its bearing, 2 screws', [('head roll rear hub', X)], [r'^head roll hub screw']),
        ('The bus board down onto them, 4 screws. Its leads on first: servo sockets, power terminals, ' + ('and the 3 UART wires soldered to its header, the pins snipped to 3 mm; both jumpers at A' if G.RECIPE == 'dynamixel' else 'USB-C'), [('Waveshare bus board', NEG(Z))], [r'^bus board top screw']),
        *([] if BODY_PI else [('The Pi onto its plate from the front, ' + ('its 5 wires soldered first: 5 V and ground on pins 2 and 6, GPIO 14 and 15 and ground on pins 8, 10 and 14. Its microSD card in, 4 screws, the camera ribbon out of its socket' if G.RECIPE == 'dynamixel' else 'its USB ports up and its microSD card in, 4 screws. Then its 2 right-angle plugs, and the camera ribbon out of its socket'), [('Pi Zero 2 W', NEG(X))], [r'^Pi screw'])]),
        ('The 5 V USB converter on edge on the base beside the Pi, foam tape: its input to the green terminals, ' + ("its USB socket's 5 V and ground pins soldered to the Pi's 2 wires from pins 2 and 6. Measure 5 V first" if G.RECIPE == 'dynamixel' else 'a USB cable to the PWR port of the Pi'), [('head 5 V regulator', NEG(Z))], []),
        ('The U2D2 on the frame, from below, foam tape', [('U2D2', Z)], []),
        ('The Feetech bus board on its rails, foam tape', [('servo bus board', NEG(Z))], []),
        ('The Feetech board onto the plate at the back of the head, jumpers at A, 4 screws. Its neck lead, UART wires and power pairs on first', [('leg bus board', X)], [r'^leg bus board screw']),
        ('The jaw, up from below, its tab sliding onto the horn, 4 screws', [('@jawbench', Z)], [r'^beak fork screw']),
        ('The hood, down between the jaw\'s tabs', [('hood', NEG(Z))], []),
        ('Open the jaw and glue in the soft mouth roof through the gap', [('mouth roof', Past(Z, ['jaw', 'jaw soft insert']))], []),
        ('The camera, its ribbon to the Pi, then glue in the face (the camera taped to its back), the eye ring and its iris', [('camera', NEG(X)), ('face', NEG(X)), ('eye ring', NEG(X)), ('eye iris', NEG(X))], []),
        ('2 screws up through the band into the hood\'s back', [], [r'^hood rear screw']),
        ('A 0.5 mm washer, and the hinge screw on the far side holds the jaw and the hood', [], [r'^[LR] hinge screw']),
    ])]
    return out


def sections_for(B):
    """sections(), without the steps of parts this variant doesn't have."""
    out = []
    for sec in sections():
        steps = []
        for cap, items, pats in sec['steps']:
            keep = [(i, w) for i, w in items if i.startswith('@') or i in B.parts]
            if keep or not items:
                steps.append((cap, keep, pats))
        out.append(dict(sec, steps=steps))
    return out


class Build:
    def __init__(self, spec, S=None):
        self.spec, self.S = spec, S or C.Solids(spec)
        self.idx = {g['name']: (ln, i, g) for ln, l in spec['links'].items() for i, g in enumerate(l['geoms'])}
        # PARTS names what the first variant has. A variant's servos and battery have their own names, and a part it doesn't draw isn't
        # in it: ALL still fails on any rigid geom left in no part
        def real(n):
            if n in self.idx:
                return n
            for k in (' actuator (buy) ', 'battery '):
                if k in n:
                    head = n.split(k)[0] + k
                    return next((x for x in self.idx if x.startswith(head)), None)
            return None
        self.parts = {}
        for n, k, gs in (spec.get('parts') or PARTS):   # (a spec with its own parts list: spec/nano_duck.py's)
            have = [x for x in (real(g) for g in gs) if x]
            if have:
                self.parts[n] = dict(kind=k, geoms=have)
        for g in self.idx:                             # a boss goes with the part that holds the plate over that horn
            if g.endswith(' seat boss') and not any(g in p['geoms'] for p in self.parts.values()):
                n_, side = g[:-len(' seat boss')].rsplit(' ', 1)
                holder = next((k for k, p in self.parts.items() if f'{n_} {side} boss shoulder' in p['geoms']), None)
                if holder is None and 'ankle' in n_:
                    holder = f'{n_[0]} shoe'
                if holder:
                    self.parts[holder]['geoms'].append(g)
        for g in self.idx:                             # a servo's plugs go in with it
            if ' cable plugs ' in g:
                j = g.split(' cable plugs')[0]
                servo = f"{j.replace(' ankle pitch', ' ankle').replace('neck pitch', 'neck')} servo"
                if servo in self.parts and g not in self.parts[servo]['geoms']:
                    self.parts[servo]['geoms'].append(g)
        self.owner = {g: n for n, p in self.parts.items() for g in p['geoms']}
        self.screws = {g['name']: (ln, g) for ln, l in spec['links'].items() for g in l['geoms'] if 'fast' in g}
        self.cuts = {}
        for l in spec['links'].values():
            for g in l['geoms']:
                for t in g.get('targets', []) if g['role'] == 'cut' else []:
                    self.cuts.setdefault(t, []).append(g)

    def inside(self, g, pts):
        """Points in a part, its cuts removed. A shell is its wall: a screw inside the body isn't in the body shell. (Taking it as solid drilled a hole through
        the shell and covers over every screw under them, and counted air as thread; only the first Fusion picture showed it.)"""
        hit = G.inside(g, pts)
        for c in self.cuts.get(g['name'], []):
            hit = hit & ~G.inside(c, pts)
        if g['role'] == 'shell':
            hit = hit & ~G.inside(G.inner_of(g, C.WALL), pts)
        return hit

    def solid(self, part):
        ms = []
        for g in self.parts[part]['geoms']:
            ln, i, g_ = self.idx[g]
            if g_.get('wire'):                         # wires bend round what they pass: they need a way out, not a straight path in
                continue
            ms.append(self.S.wall.get((ln, i), self.S.geom[(ln, i)]))
        return C.union(ms)


def tube(p, d, r0, L):
    import trimesh
    T = np.eye(4)
    T[:3, :3] = G._frame_to(np.asarray(d, float))
    T[:3, 3] = np.asarray(p) + np.asarray(d) * (r0 + L / 2)
    m = trimesh.creation.cylinder(radius=KEY_R, height=L, sections=12)
    m.apply_transform(T)
    return C.to_manifold(m)


def check_hold(B, rep):
    """(2026-11-03: the hood hung on 2 hinge screws on one axis, and would have tipped about them. Every check passed it.)"""
    lines = {n: [] for n, p in B.parts.items() if p['kind'] == 'P'}
    for n, (ln, g) in B.screws.items():
        f = g['fast']
        p, d = np.array(f['p']), np.array(f['d'])
        for t in np.linspace(0, f['L'], 25):
            q = (p + d * max(t, 0.0002))[None]
            for part in lines:
                if any(B.idx[x][0] == ln and B.inside(B.idx[x][2], q).any() for x in B.parts[part]['geoms']):
                    if not any(np.allclose(p, a[0]) for a in lines[part]):
                        lines[part].append((p, d))
    for part, ls in lines.items():
        glued = GLUED | set(B.spec.get('glued', []))
        if part in glued or part in PIVOTS:
            rep.add('HOLD', part, True, 'glued' if part in glued else 'turns on its screws')
            continue
        if part in B.spec.get('splined', []) and len(ls) == 1:   # a hub on a servo's spline can't turn about its one horn screw
            rep.add('HOLD', part, True, 'on its servo\'s spline, held by the horn screw')
            continue
        if len(ls) < 2:
            rep.add('HOLD', part, False, f'{len(ls)} screw: it can turn about it')
            continue
        p0, d0 = ls[0]
        off = [np.linalg.norm(np.cross(a - p0, d0)) for a, _ in ls[1:]]
        rep.add('HOLD', part, max(off) > 0.001, f'{len(ls)} screws' if max(off) > 0.001 else f'{len(ls)} screws, all on one axis: it can turn about them')


# parts that are meant to share space: a horn and the plate on it, a part seated in its neighbour
SEATS = (' flange', ' rear boss', ' rear horn', 'bearing', ' cradle', ' shoulder', ' seat', ' seat boss')


def check_apart(B, rep):
    """Bought parts of one link against everything else on it. (2026-11-03: a printed bar ran through the Pi's board, and the static
    check skips parts of one link.)"""
    names = [n for n, p in B.parts.items()]
    for a in names:
        pa = B.parts[a]
        if pa['kind'] != 'B' or any(k in a for k in (' servo', 'horn', 'bearing')) and 'servo' not in a:
            continue
        la = {B.idx[g][0] for g in pa['geoms']}
        sa = B.solid(a)
        for b in names:
            if b == a or not ({B.idx[g][0] for g in B.parts[b]['geoms']} & la):
                continue
            if B.parts[b]['kind'] == 'B' and b <= a and b in names:
                pass
            v = C.overlap(sa, B.solid(b))
            # (a servo's plates are drawn up to 0.5 mm into its case, so one solid joins them: tools/export_prints.py cuts the real pocket, with clearance)
            ok = v <= (0.9e-6 if ' servo' in a else C.SEATED)
            if not ok:
                rep.add('APART', f'{a} / {b}', False, f'{v * 1e6:.2f} cm3 in the same place')
    rep.add('APART', 'bought parts', True)
    # a cable has to plug in: nothing printed where the plugs go (limits above would pass a whole plate over them)
    for n, (ln, i, g) in B.idx.items():
        if 'cable plugs' not in n:
            continue
        plug = B.S.geom[(ln, i)]
        hit = [p for p, q in B.parts.items() if q['kind'] == 'P' and C.overlap(plug, B.solid(p)) > C.TOUCH]
        rep.add('APART', n, not hit, f'no way to plug the cable in: {hit} in the way' if hit else '')


def horn_hole(spec, g):
    """A horn, as a screw meets it: as deep as its real holes go, not as high as it stands. (An STS3215's holes run on below the box's plane, where the case's drawing is.)"""
    if g['name'].endswith(' rear horn'):           # the same horn on the rear: only where a plug's room stands its plate off, so the screw reaches further in. Its holes go as deep
        j = next((x for x in spec['joints'] if x['name'] == g['name'][:-len(' rear horn')].replace(' ', '_')), None)
        if j and j.get('rear_stack'):
            A_ = spec['actuators'][j['act']]
            ix = 'xyz'.index(j['axis'])
            c = list(j['act_c'])
            c[ix] -= j['out'] * (A_['box'][2] / 2 + A_['horn_top'] - A_['horn_hole_depth'] / 2)
            return dict(g, c=c, h=A_['horn_hole_depth'])
        return g
    if not g['name'].endswith(' flange'):
        return g
    j = next(x for x in spec['joints'] if x['name'] == g['name'][:-len(' flange')])
    A_ = spec['actuators'][j['act']]
    if 'horn_hole_depth' not in A_ or j['remote']:   # (a remote servo's joint origin isn't its horn)
        return g
    real = A_['horn_top']
    ix = 'xyz'.index(j['axis'])
    c = list(j['origin'])
    top = c[ix] - j['out'] * (j['stack'] - real)
    c[ix] = top - j['out'] * A_['horn_hole_depth'] / 2
    return dict(g, c=c, h=A_['horn_hole_depth'])


# where a servo is held at its far end only, and why (the rest must have a screw at each end)
ONE_END = {'neck pitch': 'the fork turns over the near end, both faces', 'head pitch': 'the fork turns over the near end, both faces',
           'head roll': 'a bearing on both faces, and the fork round them',
           'L ankle pitch': 'the rear idler is screwed to the same plate, so that end is held by it', 'R ankle pitch': 'the rear idler is screwed to the same plate, so that end is held by it'}


def check_ends(B, rep):
    """A servo screwed down at one end of its case only: each screw has a reason, and no reach check sees the missing ones. (2026-11-04, the user.)"""
    for j in B.spec['joints']:
        A_ = B.spec['actuators'][j['act']]
        n = j['name'].replace('_', ' ')
        ix = 'xyz'.index(j['long'][1])
        U = np.zeros(3)
        U[ix] = 1 if j['long'][0] == '+' else -1
        c = np.array(j['act_c']) + U * A_['shaft_x']
        us = [float((np.array(g['fast']['p']) - c) @ U) for k, (ln, g) in B.screws.items() if k.startswith(f'{n} cradle screw')]
        near = [u for u in us if u < -0.002]
        if near:
            rep.add('ENDS', n, True, f'{len(near)} of {len(us)} screws at the near end')
        elif n in ONE_END:
            rep.add('ENDS', n, True, f'far end only: {ONE_END[n]}')
        else:
            rep.add('ENDS', n, False, f'{len(us)} screws, all at the far end: nothing holds the near end')


def check_horn_tips(B, rep):
    """A horn whose holes go right through it, as the HD-1910's: 2.5 mm of disc, 0.5 mm of air, then the case. A screw that's too long doesn't bite more: its tip drags on the case as the joint turns. Only for such a servo ('centre_screw'). --bad horntip."""
    for j in B.spec['joints']:
        A_ = B.spec['actuators'][j['act']]
        if 'centre_screw' not in A_ or j['remote']:
            continue
        n = j['name'].replace('_', ' ')
        ix = 'xyz'.index(j['axis'])
        H, real = A_['box'][2], A_['horn_top']
        tops = {'horn': j['act_c'][ix] + j['out'] * (H / 2 + real), 'rear': j['act_c'][ix] - j['out'] * (H / 2 + real)}
        for k, (ln, g) in B.screws.items():
            side = 'horn' if k.startswith(f'{n} fork screw horn') else 'rear' if k.startswith(f'{n} fork screw rear') or (k.startswith(f'{n[0]} ankle boss screw') and 'ankle' in n) else None
            if side is None:
                continue
            f = g['fast']
            tip = f['p'][ix] + f['d'][ix] * f['L']
            into = (tops[side] - tip) * j['out'] * (1 if side == 'horn' else -1)
            room = real - 0.0002
            rep.add('HORNTIP', k, 0 < into <= room, f'{into * 1000:.1f} mm below the horn\'s face, {room * 1000:.1f} of room above the case')


def check_seat(B, rep):
    """Does a printed part reach the real horn, where its screws go? (A horn drawn tall enough to reach the plate passed every check: THREAD and FIT measured the drawing.)"""
    for j in B.spec['joints']:
        if j['remote']:
            continue
        A_ = B.spec['actuators'][j['act']]
        real = A_.get('horn_top', j['stack'])
        if j['stack'] <= real + 1e-9:
            continue
        n = j['name'].replace('_', ' ')
        ix = 'xyz'.index(j['axis'])
        e = np.zeros(3)
        e[ix] = j['out']
        a, b = np.zeros(3), np.zeros(3)
        a[(ix + 1) % 3], b[(ix + 2) % 3] = 1, 1
        horn_link = j['parent'] if j.get('case_in') == 'child' else j['child']
        o = np.array(j['origin']) - e * (j['stack'] - real - 0.0001)
        r = A_['horn_hole_r']
        pts = np.array([o + r * (np.cos(t) * a + np.sin(t) * b) for t in np.linspace(0, 2 * np.pi, 8, endpoint=False)])
        hit = np.zeros(len(pts), bool)
        for q, (ln, i, g) in B.idx.items():
            if ln == horn_link and g['role'] == 'struct' and B.parts.get(B.owner.get(q), {}).get('kind') == 'P':
                hit |= B.inside(g, pts)
        rep.add('SEAT', n, hit.all(), '' if hit.all() else f'nothing printed on the real horn at {(~hit).sum()} of 8 points: the screws span a gap')


EDGE_MIN = 0.0008                                  # plastic round a screw's clearance hole: 2 perimeters of a 0.4 mm nozzle. It clamps, it doesn't thread: 3 would suit a pilot hole
CLEAR_R = {'M2': 0.00115, 'M2.5': 0.00145, 'M3': 0.0017}


def check_bought(B, rep):
    """A bought part's solid can't be in a printed part's list: it gets fused into the STL. (2026-11-04: the ankle bearing's outer race was listed in the inner ear, so the ear printed with a 12.5 mm hole for a 15 mm bearing, and every fit check passed as the race "fit".)"""
    for n, p_ in B.parts.items():
        if p_['kind'] != 'P':
            continue
        steel = [x for x in p_['geoms'] if B.idx[x][2].get('material') == 'steel']
        rep.add('BOUGHT', n, not steel, '' if not steel else f'{steel[:3]} would print as plastic, fused into this part')


# printed parts allowed more than one colour, and why. Anything else is one filament.
TWO_COLOURS = {'hood': 'its orange band is painted or split in the slicer: prints/README.md', 'face': 'the sensor window is a raised pad in the same colour, dark only if painted',
               'L shoe': 'the seat boss is hidden under the horn', 'R shoe': 'the seat boss is hidden under the horn'}


def check_colour(B, rep):
    """One STL is one colour unless someone has said how to print it. (2026-11-04, the user: the head base "looks like single piece but two colors": it was, and no note said so.)"""
    for n, p_ in B.parts.items():
        if p_['kind'] != 'P':
            continue
        cols = {B.idx[x][2]['color'] for x in p_['geoms'] if B.idx[x][2]['role'] != 'decal'}
        ok = len(cols) <= 1 or n in TWO_COLOURS
        rep.add('COLOUR', n, ok, '' if ok else f'{sorted(cols)} in one STL: print note, or one colour')


def check_pierce(B, rep):
    """What a shell gives way to, a servo's pocket or a part inside it, with its clearance, must stay inside the shell: a corner past the outer surface is a hole in the finish. (2026-11-04: a triangular hole in the hood, seen only in a picture, from a small tray wall above the beak servo. The collision check passed it.)"""
    GAP_ = 0.00015
    for n, (ln, i, g) in B.idx.items():
        if g['role'] not in ('actuator', 'struct') or 'size' not in g or g['kind'] != 'rbox' or B.owner.get(n) is None:
            continue
        lo, hi = np.array(g['c']) - np.array(g['size']) / 2 - GAP_, np.array(g['c']) + np.array(g['size']) / 2 + GAP_
        corners = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
        for sn, (l2, i2, s_) in B.idx.items():
            if l2 != ln or s_['role'] != 'shell':
                continue
            m = C.to_manifold(G.mesh_of(s_))
            b = m.bounding_box()
            box_lo, box_hi = np.array(b[:3]), np.array(b[3:])
            within = ((corners > box_lo + 0.001) & (corners < box_hi - 0.001)).all(axis=1)
            filled = G.inside(s_, corners)
            for c_ in B.cuts.get(sn, []):              # an opening the shell is meant to have isn't a hole
                filled = filled | G.inside(c_, corners)
            bad = within & ~filled
            # only where the case is otherwise inside this shell: most of it in, a corner out
            if G.inside(s_, np.array([g['c']])).any() and bad.any() and B.owner.get(n) != B.owner.get(sn):
                rep.add('PIERCE', f'{n} / {sn}', False, f'{bad.sum()} corner(s) of its pocket past the outer surface: a hole in the {sn}')
    rep.add('PIERCE', 'servo pockets', True)


def check_edge(B, rep):
    """A screw hole too near a part's edge splits when it's tightened. (2026-11-04, the user circled one with 0.4 mm of plastic: it passed WALL, HELD and THREAD.)"""
    for n, (ln, g) in B.screws.items():
        f = g['fast']
        p, d = np.array(f['p']), np.array(f['d'])
        if f.get('pivot'):
            continue
        q0 = p + d * 0.0002
        owners = {B.owner.get(x) for x, (l2, i2, g2) in B.idx.items() if l2 == ln and g2['role'] in ('struct', 'shell') and B.inside(g2, q0[None]).any()}
        owners = [o for o in owners if o and B.parts[o]['kind'] == 'P']
        if not owners:
            continue
        a = np.cross(d, [1.0, 0, 0] if abs(d[0]) < 0.9 else [0, 1.0, 0])
        a /= np.linalg.norm(a)
        b = np.cross(d, a)
        r = CLEAR_R[f['size'].split()[0]] + EDGE_MIN
        ring = np.array([q0 + r * (np.cos(t) * a + np.sin(t) * b) for t in np.linspace(0, 2 * np.pi, 24, endpoint=False)])
        hit = np.zeros(len(ring), bool)
        for o in owners:
            for x in B.parts[o]['geoms']:
                hit |= B.inside(B.idx[x][2], ring)
        if ' cradle screw near ' in n and hit.mean() > 0.8:   # an STS3215's near holes, in the floor under a bearing: it runs out toward the bore, not an edge, and the bearing's press sits on it. Print one and look
            rep.add('EDGE', n, True, f'{(~hit).mean() * 100:.0f} % of the ring runs into the bearing bore: under the race, watch it on the first print')
            continue
        ok = hit.mean() > 0.95
        rep.add('EDGE', n, ok, '' if ok else f'{(~hit).mean() * 100:.0f} % of the ring {EDGE_MIN * 1000:.1f} mm round its hole is outside {owners[0]}: it would split')


def check(spec, secs, rep, S=None):
    B = Build(spec, S)
    placed, used, done = {}, set(), {}
    for sec in secs:
        have = [p for o in sec['on'] for p in done[o]]
        for cap, items, pats in sec['steps']:
            if items and all(not i.startswith('@') and i not in B.parts for i, _ in items):
                continue                           # a step for parts this variant doesn't have
            for item, way in items:
                group = done[item[1:]] if item.startswith('@') else [item]
                if not item.startswith('@'):
                    if item not in B.parts:
                        continue                   # a part this variant doesn't have
                    placed[item] = placed.get(item, 0) + 1
                if way is not None and have:
                    fixed = C.union([B.solid(p) for p in have if p not in getattr(way, 'aside', ())])
                    moving = C.union([B.solid(p) for p in group])
                    base = C.overlap(moving, fixed)
                    worst = 0.0
                    for t in np.arange(0.001, getattr(way, 'travel', TRAVEL) + 1e-9, 0.001):
                        v = C.overlap(moving.translate([-a * t for a in way]), fixed)
                        worst = max(worst, v - base)
                    who = []
                    if worst > C.TOUCH:
                        for o in [p for p in have if p not in getattr(way, 'aside', ())]:
                            so = B.solid(o)
                            b0 = C.overlap(moving, so)
                            if max(C.overlap(moving.translate([-a * t for a in way]), so) - b0
                                   for t in np.arange(0.002, getattr(way, 'travel', TRAVEL) + 1e-9, 0.002)) > C.TOUCH:
                                who.append(o)
                    rep.add('PATH', f"{sec['key']}: {item}", worst <= C.TOUCH,
                            f'cuts {worst * 1e6:.2f} cm3 into {who}, travelling {way}' if worst > C.TOUCH else '')
                have = have + list(group)
            for pat in pats:
                names = [n for n in B.screws if re.search(pat, n)]
                if not names:
                    rep.add('ALL', f'pattern {pat}', False, 'matches no screw')
                for n in names:
                    used.add(n)
                    ln, g = B.screws[n]
                    f = g['fast']
                    p, d = np.array(f['p']), np.array(f['d'])
                    # HELD: a part under the head
                    # (or, on a hobby servo's ear screw, the ear itself: that is what it clamps down onto the printed rim)
                    under = [q for q, (l2, i2, g2) in B.idx.items() if l2 == ln and (g2['role'] in ('struct', 'shell') or f.get('ear') and g2['role'] == 'actuator')
                             and B.inside(g2, (p + d * 0.0002)[None]).any()]
                    clamp = {B.owner.get(q) for q in under}
                    if not f.get('pivot'):
                        rep.add('HELD', n, bool(under), '' if under else 'nothing under the head to clamp')
                    # THREAD: the tip half in something else
                    t0 = 0.0 if f['L'] < 0.005 else f['L'] / 2     # (a short screw in a recess, as the Koch arm's: 3.6 mm long, its tip half could never bite the 2 mm asked)
                    ts = np.linspace(t0, f['L'], 21)
                    pts = p + np.outer(ts, d)
                    bite = np.zeros(len(pts), bool)
                    for q, (l2, i2, g2) in B.idx.items():
                        if l2 == ln and g2['role'] in G.RIGID and B.owner.get(q) not in clamp:
                            bite |= B.inside(horn_hole(spec, g2), pts)
                    mm = bite.mean() * (f['L'] - t0)
                    rep.add('THREAD', n, mm >= MIN_BITE - 1e-4, f"{f['size']} x {f['L'] * 1000:.0f}: bites {mm * 1000:.1f} mm")
                    # ACCESS: a key from the head, past parts on now (its own counterbore is printed to suit)
                    key = tube(p, -d, 0.0016, KEY_REACH - 0.0016)
                    blocked = [o for o in have if o not in clamp and C.overlap(key, B.solid(o)) > 1e-9]
                    rep.add('ACCESS', n, not blocked, f'blocked by {blocked[:3]}' if blocked else '')
        done[sec['key']] = have
    check_hold(B, rep)
    check_apart(B, rep)
    check_ends(B, rep)
    check_bought(B, rep)
    check_colour(B, rep)
    check_edge(B, rep)
    check_pierce(B, rep)
    check_seat(B, rep)
    check_horn_tips(B, rep)
    for n, p in B.parts.items():
        c = placed.get(n, 0)
        rep.add('ALL', f'part {n}', c == 1, '' if c == 1 else f'fitted {c} times')
    for ln, l in spec['links'].items():
        for g in l['geoms']:
            if g['role'] in G.RIGID and g['name'] not in B.owner:
                rep.add('ALL', f"geom {g['name']}", False, 'in no part')
    for n in B.screws:
        if n not in used:
            rep.add('ALL', f'screw {n}', False, 'never fitted')
    return B


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--bad')
    a = ap.parse_args()
    spec = G.load()
    # (a spec with its own order: spec/nano_duck.py's. Its ways are lists, which the checks take as they take tuples)
    # (a 4th number: Short, checked for that many mm)
    way = lambda w: None if not w else Short(tuple(w[:3]), w[3] / 1000) if len(w) > 3 else tuple(w)
    secs = [dict(s_, steps=[(c, [(p, way(w)) for p, w in it], pats) for c, it, pats in s_['steps']]) for s_ in spec['assembly']] if 'assembly' in spec else sections()
    if a.bad == 'seal':                            # the yaw bracket's screws after its roll servo is in
        body = next(s for s in secs if s['key'] == 'body')
        st = body['steps']
        k = next(i for i, x in enumerate(st) if 'yaw brackets' in x[0])
        cap, items, pats = st[k]
        st[k] = (cap, items, [])
        st.append(('late', [], pats))
    elif a.bad == 'thread':                        # a horn screw far too short
        for l in spec['links'].values():
            for g in l['geoms']:
                if g['name'].startswith('L knee fork screw') and 'fast' in g:
                    g['fast']['L'] = 0.003
    elif a.bad == 'horntip':                       # a horn screw 2 mm too long: an HD-1910's tip on its case
        for l in spec['links'].values():
            for g in l['geoms']:
                if g['name'].startswith('L knee fork screw') and 'fast' in g:
                    g['fast']['L'] += 0.002
    elif a.bad == 'order':                         # a part forgotten
        body = next(s for s in secs if s['key'] == 'body')
        body['steps'] = [x for x in body['steps'] if 'push in the hip yaw' not in x[0]]
    elif a.bad == 'path':                          # the lids pushed in from the wrong side, through their servos
        th = next(s for s in secs if s['key'] == 'thigh_L')
        th['steps'] = [(c, [(p, NEG(w)) if 'lid' in p else (p, w) for p, w in it], s_) for c, it, s_ in th['steps']]
    elif a.bad == 'hold':                          # the hood on its hinge screws alone
        for g in spec['links']['head']['geoms']:
            if g['name'].startswith('hood rear screw'):
                g['fast']['L'] = 0.001
    elif a.bad == 'window':                        # the thigh plate over the plugs, as it was
        for ln in spec['links']:
            spec['links'][ln]['geoms'] = [g for g in spec['links'][ln]['geoms'] if not g['name'].endswith('plug window')]
    elif a.bad == 'edge':                          # a screw moved to its plate's edge
        for g in spec['links']['L_hip_pitch_link']['geoms']:
            if g['name'].startswith('L hip pitch cradle screw rear near') and 'fast' in g:
                g['fast']['p'][0] += 0.0030
    elif a.bad == 'ends':                          # the near-end screws gone
        for ln in spec['links']:
            spec['links'][ln]['geoms'] = [g for g in spec['links'][ln]['geoms'] if ' cradle screw near' not in g['name'] and ' cradle screw rear near' not in g['name']]
    elif a.bad == 'seat':                          # no boss: the plate over a bearing, in the air above the horn
        for ln in spec['links']:
            spec['links'][ln]['geoms'] = [g for g in spec['links'][ln]['geoms'] if ' seat boss' not in g['name']]
    elif a.bad == 'apart':                         # the bar back through the Pi
        for g in spec['links']['head']['geoms']:
            if g['name'] == 'Pi mount':
                g['size'][0] += 0.02
    rep = C.Report()
    check(spec, secs, rep)
    fails = rep.failing()
    if a.bad:
        print(f"--bad {a.bad}: {'CAUGHT' if fails else 'NOT CAUGHT'} ({len(fails)} failing)")
        for r in fails[:3]:
            print('  ', r['group'], r['item'], r['detail'])
        return 0 if fails else 1
    rep.write(G.out_path('reports', 'assembly.md'), 'From tools/assembly.py')
    for r in fails:
        print(r['group'], r['item'], r['detail'])
    print(f'{len(rep.rows)} assembly checks, {len(fails)} failing')
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
