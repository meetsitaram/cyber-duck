"""The big duck's BOM: docs/big/BOM.md, counted from spec/robot_big.json. MILLI_RECIPE=big python -I tools/make_docs.py calls it.

Its own file, as nothing in it fits a small duck's: plates, bearings and a battery pack that no shop sells ready. Prices and links were read on 2026-11-05. A price nobody could read stays empty. Never invent one.
"""
import collections
import os

import numpy as np

import geom as G
import mass_model as MM

C = MM.C
UNSAFE = ('**NOT SAFE TO BUILD.** This is a design study. It would stand 1.8 m and weigh about 100 kg, with no stress analysis, metal frames nobody has checked, no real actuator measured, and no brakes: its joints let go when the power does. '
          'A failure could seriously hurt someone. Don\'t order parts for it, or build any of it, until everything under [Open before ordering](#open-before-ordering) is closed, by someone able to sign off a machine of this size.')
HOLES = {'RS04': (10, 'M4', 9, 'M5'), 'RS03': (4, 'M4', 6, 'M4'), 'RS00': (6, 'M3', 6, 'M3')}     # the mounting and output screws, from each STEP file. The RS03 has 4 free on its rear; the RS00's mount isn't designed


def link(label, url):
    return f'[{label}]({url})'


def bought(spec):
    """What the model holds: actuators, bearings, modelled screws."""
    act, bear, screws = collections.Counter(), collections.Counter(), collections.Counter()
    for l in spec['links'].values():
        for g in l['geoms']:
            if g['role'] == 'actuator':
                act[g['name'].split()[-1]] += 1
            if g.get('part') and 'outer race' in g['name']:
                bear[g['part']] += 1
            if 'fast' in g:
                screws[(g['fast']['size'], round(g['fast']['L'] * 1000))] += 1
    return act, bear, screws


def fabricated(spec):
    """Every metal part and printed panel, from its solid. Left and right legs are one row with a quantity of 2. Returns (metal rows, printed rows, masses by material)."""
    S = C.Solids(spec)
    metal, printed, by = [], [], collections.Counter()
    for ln, l in spec['links'].items():
        if ln.startswith('R_'):
            continue
        q = 2 if ln.startswith('L_') else 1
        mass = {p[0]: p[2] for p in MM.parts(spec, S, ln)}
        for i, g in enumerate(l['geoms']):
            m = S.geom.get((ln, i))
            if m is None or g['role'] in ('cut', 'actuator') or g.get('part') or g.get('seat') or 'fast' in g or '(buy)' in g['name'] or g.get('material') not in ('al7075', 'al6061', 'petg', 'tpu', 'steel'):
                continue
            v = np.asarray(m.to_mesh().vert_properties[:, :3], float)
            if not len(v):
                continue
            dims = np.sort(np.ptp(v, axis=0)) * 1000
            kg = mass.get(g['name'], 0.0)
            by[g['material']] += kg * q
            name = g['name'][2:] if g['name'][:2] in ('L ', 'R ') and q == 2 else g['name']
            row = dict(name=name, q=q, mat=g['material'], dims=dims, kg=kg, fill=g.get('fill', 1.0), link=ln)
            (printed if g['material'] in ('petg', 'tpu') else metal).append(row)
    return metal, printed, by


def process(r):
    if r['mat'] == 'steel':
        return 'turned'
    if r['dims'][0] <= 8.01:
        return f"cut from {r['dims'][0]:.0f} mm sheet"
    if r['dims'][0] <= 14 and r['dims'][1] <= 90:
        return 'turned or milled'
    return 'milled from solid; pockets not designed'


def bom_big(spec):
    act, bear, screws = bought(spec)
    metal, printed, by = fabricated(spec)
    inert = MM.link_inertials(spec)
    total = sum(v[0] for v in inert.values())
    n04, n03, n00 = act['RS04'], act['RS03'], act['RS00']
    hole = collections.Counter()
    for a, n in act.items():
        k, s, k2, s2 = HOLES[a]
        hole[s] += k * n
        hole[s2] += k2 * n
    modelled = collections.Counter()
    for (s, _), n in screws.items():
        modelled[s] += n
    A_ = link('Robotics Center', 'https://www.roboticscenter.ai/store/product/robstride-04')
    A3 = link('Robotics Center', 'https://www.roboticscenter.ai/store/product/robstride-03')
    A0 = link('Robotics Center', 'https://www.roboticscenter.ai/store/product/robstride-00')
    OE3 = link('OpenELAB', 'https://openelab.io/products/robstride03-qdd-60n-m-integrated-joint-motor-module')
    OP = link('OpenELAB', 'https://openelab.io/products/robstride-power-supply-board')
    FBJ = link('Quality Bearings Online', 'https://www.qualitybearingsonline.ca/6908-2rs-fbj-thin-section-bearing-40x62x12mm/')
    Q61 = link('Quality Bearings Online', 'https://www.qualitybearingsonline.ca/61824-2rsr-fag-thin-section-120x150x16mm/')
    TX = link('Texas Belting', 'https://texasbelting.com/products/bb-40a25')
    Z13 = link('Zoro', 'https://www.zoro.com/bb-manufacturing-40b13-roller-chain-sprocket-steel-40b13/i/G905366475/')
    HUB = link('RCDrone', 'https://rcdrone.top/products/robstride-usb-can-hub-module')
    HUB2 = link('AiFitLab', 'https://aifitlab.com/products/robstride-usb-can-hub-module')
    AD = link('Adafruit 4754', 'https://www.adafruit.com/product/4754')
    # (key, part, maker and part number, specification, qty per robot, order unit, order qty, unit price or None, source, Required / Optional, Amazon search words)
    GROUPS = [
        ('Actuators', [
            ('rs04', 'Joint actuator', 'RobStride RS04, CAN 2.0', f'48 V (24-60 V), 40 N m rated on a 345 x 345 mm heat sink, 35 on 220 x 200, 120 N m peak for 3 s, 1.42 kg, 9:1. Hips, knees, ankles, neck, head pitch', n04, 'each', n04, 295.00, A_ + ': backorder, 4-6 weeks. Shipping about $67 each by its estimate', 'Required', 'RobStride RS04 actuator'),
            ('rs03', 'Joint actuator', 'RobStride RS03, CAN 2.0', '48 V (24-60 V), 20 N m rated, 60 peak, 0.88 kg, 9:1. Head yaw, head roll', n03, 'each', n03, 245.00, A3 + ': backorder, 4-6 weeks. ' + OE3 + ' lists it at 226.75 EUR, sold out', 'Required', 'RobStride RS03 actuator'),
            ('rs00', 'Joint actuator', 'RobStride RS00, CAN 2.0', '48 V (24-60 V), 5 N m rated, 14 peak, 0.31 kg, 10:1. Beak', n00, 'each', n00, 185.00, A0 + ': backorder, 4-6 weeks', 'Required', 'RobStride RS00 actuator'),
            ('spare', 'Spare RS04', 'RobStride RS04, CAN 2.0', 'one actuator to test on a bench first, and to measure', 1, 'each', 1, 295.00, A_, 'Optional', 'RobStride RS04 actuator'),
        ]),
        ('Bearings', [
            ('b61824', 'Thin-section bearing, hip yaw', '61824', '120 x 150 x 16 mm', bear['61824'], 'each', bear['61824'], None, Q61 + ' lists the sealed 2RSR, no price read. The only price found, $218.82, is a different grease version', 'Required', '61824 bearing 120x150x16mm'),
            ('b6908', 'Bearing, knee and head pitch', '6908', '40 x 62 x 12 mm', bear['6908'], 'each', bear['6908'], None, FBJ + ': 6908-2RS at $22.17, Canadian shop, currency not confirmed', 'Required', '6908 2RS bearing 40x62x12mm'),
            ('b6206', 'Bearing, ankle far side', '6206', '30 x 62 x 16 mm', bear['6206'], 'each', bear['6206'], None, 'any bearing shop; no price read', 'Required', '6206 2RS bearing 30x62x16mm'),
        ]),
        ('Head pitch chain', [
            ('s13', 'Sprocket, drive', 'B&B 40B13, or plain-bore equivalent', '#40 chain, 13 teeth, 53.1 mm pitch circle. Needs a hub machined to the RS04 output: not designed', 1, 'each', 1, 13.99, Z13, 'Required', '40B13 sprocket 13 tooth'),
            ('s25', 'Sprocket, joint', 'B&B 40A25, or plate equivalent', '#40 chain, 25 teeth, 101.3 mm pitch circle. Needs boring to the 40 mm shaft and a key or clamp: not designed', 1, 'each', 1, 21.43, TX, 'Required', '40A25 sprocket 25 tooth'),
            ('ch', 'Roller chain', 'ANSI #40, single strand', '12.7 mm pitch, about 52 links for these centres, 205 mm. 52 links does not land exactly on 205, so it needs the tensioner that is still not designed', 1, 'length', 1, None, 'any power transmission shop', 'Required', 'ANSI 40 roller chain'),
            ('cl', 'Connecting link', 'ANSI #40', '', 1, 'each', 1, None, 'with the chain', 'Required', 'ANSI 40 connecting link'),
        ]),
        ('Fasteners', [
            ('m4', 'Socket cap and countersunk screws, M4: actuator mounts, RS03 outputs and front rings, frame joints', 'class 12.9 into actuators; 8.8 or A4-70 in the frame, to be decided', f"M4, {modelled['M4']} in the model. Lengths by joint: [the assembly's screw table](ASSEMBLY.md#screws)", modelled['M4'], 'box of 100', int(np.ceil(modelled['M4'] / 100)), None, 'any fastener shop', 'Required', 'M4 socket cap screw 12.9'),
            ('m5', 'Socket cap and countersunk screws, M5: RS04 outputs, frame joints', 'class 12.9', f"M5, {modelled['M5']} in the model. RS04 outputs: 3 mm into a thread 3.5 deep in RobStride's file, 6.5 in its manual: see Open before ordering. Lengths: [the screw table](ASSEMBLY.md#screws)", modelled['M5'], 'box of 100', int(np.ceil(modelled['M5'] / 100)), None, 'any fastener shop', 'Required', 'M5 socket cap screw 12.9'),
            ('m3', 'Socket cap screw, M3: computer standoffs, and the RS00 (not modelled)', 'generic', f"M3, {modelled['M3']} in the model and {hole['M3']} for the RS00's mount and output", modelled['M3'] + hole['M3'], 'box of 100', 1, None, 'any fastener shop', 'Required', 'M3 socket cap screw'),
            ('eclip', 'E-clip, DIN 6799 size 8', 'spring steel', 'holds each knee rod on its 10 mm pin, in an 8 mm groove: 2 per rod', 8, 'pack of 50', 1, None, 'any fastener shop', 'Required', 'DIN 6799 E-clip 8mm'),
            ('circ', 'Circlip, DIN 471 for a 40 mm shaft', 'spring steel', 'each end of the knee shafts and the head pitch shaft', 6, 'pack of 10', 1, None, 'any fastener shop', 'Required', 'DIN 471 40mm external circlip'),
            ('key', 'Parallel key, DIN 6885 A 12 x 8', 'steel', 'knee levers and hubs on the knee shaft (4 per knee), head pitch yokes and sprocket (3). Keyways not drawn', 11, 'each', 11, None, 'any fastener shop', 'Required', 'DIN 6885 key 12x8'),
            ('insert', 'Heat-set insert, M4', 'brass', 'the shoes: 2 per piece', 8, 'pack of 50', 1, None, 'any fastener shop', 'Required', 'M4 heat set insert'),
            ('panel', 'Panel screws and inserts', 'not chosen', f'{len(printed)} printed panels have no screws or bosses designed', 0, '-', 0, None, 'not designed', 'Required', 'M3 heat set insert'),
        ]),
        ('Electronics', [
            ('pi', 'Computer', 'Raspberry Pi 5, 4 GB', 'chosen for its 2 USB 3 ports and for running a walking policy on its CPU: not tested with this one. Fits the model\'s box beside the power board, 56 + 110 mm in its 170', 1, 'each', 1, None, 'any Raspberry Pi reseller', 'Required', 'Raspberry Pi 5 4GB'),
            ('sd', 'microSD card', 'SanDisk Ultra', '32 GB microSDHC, A1, Class 10, UHS-I', 1, 'each', 1, None, '[Amazon](https://www.amazon.com/dp/B08L5HMJVW)', 'Required', 'SanDisk 32GB Ultra microSDHC A1'),
            ('hub', 'USB to CAN adapter, 3 buses', 'RobStride USB-CAN HUB module', 'USB-C to 5 CAN channels, by its listing. Whether Linux sees it as SocketCAN is not known', 1, 'each', 1, 91.25, HUB + ', from a search result: the page would not open. ' + HUB2 + ' $75, backorder, the same', 'Required', 'USB to CAN adapter 1Mbps'),
            ('usbcan', 'USB to CAN adapter, for setting IDs and firmware', 'RobStride USB-CAN module (CH340)', 'the one RobStride\'s own tool needs. DIP 2 on is its 120 ohm terminator. DIP 1 stays off', 1, 'each', 1, None, 'RobStride resellers, e.g. Pakronics; OpenELAB sells it as "Debug Board" at 20.53 EUR', 'Required', 'RobStride USB CAN debug'),
            ('imu', 'IMU', 'BNO085 module, EC Buying', 'as the small ducks, on I2C. Must sit near the pelvis, not on the power board', 1, 'each', 1, None, '[Amazon](https://www.amazon.com/dp/B0CDGZMLPP). Second source ' + AD + ' at $29.50', 'Required', 'BNO085 IMU module'),
            ('reg', '24 V to 5 V regulator for the Pi', 'not chosen', '5 V, 5 A for a Pi 5, from the board\'s 24 V rail. The board\'s own 5 V rail is 2 A rated, not enough', 1, 'each', 1, None, 'not chosen', 'Required', '24V to 5V 5A buck converter USB-C'),
            ('cam', 'USB camera', 'not chosen', 'the head\'s far from the computer, about 1.2 m through 4 joints: a ribbon won\'t reach', 1, 'each', 1, None, 'not chosen', 'Optional', 'USB camera module'),
        ]),
        ('Power and wiring', [
            ('batt', 'Battery: **candidate to bench-test, not a choice**', 'Dan-Tech Li-ion soft pack 13S5P 48V 25Ah 100A', '13S5P Samsung INR21700-50S, 54.6 V full, 1200 Wh, 100 A continuous, 180 A peak, 50 A charge, 280 x 80 x 130 mm, 5.2 kg. **No BMS, no case, no connector**', 1, 'each', 1, None, link('Dan-Tech', 'https://shop.danenergy.com/en-il/products/li-ion-battery-softpack-13s5p-48v-25ah-100a') + ': out of stock when found. A snippet gives 814 EUR, not tied to this model. See Battery options', 'Required', '13S 48V 25Ah lithium battery pack'),
            ('bms', 'Battery BMS, for the pack above', 'not chosen', '13S, at least 100 A continuous, taking back up to 50 A of braking current, and able to warn before it cuts off. **The pack must not be connected without one.** The model allows 60 x 190 x 137 mm and 2.8 kg for it, the fuse and a case', 1, 'each', 1, None, 'not chosen', 'Required', '13S BMS 100A'),
            ('board', 'Power board', 'RobStride Power Supply Board (POWER V1.3 manual), SKU ROBSTRIDE-Power', '20-65 V in. 4 XT30 branches, 20 A rated, 40 A peak each. 24 V 4 A, 12 V 2 A, 5 V 2 A. 90 x 110 x 16.5 mm, 4 x M3. Precharge, emergency stop input, CAN', 1, 'each', 1, 114.99, OP + ': $114.99 by a search result, 103.95 EUR on its page; pre-order. RCDrone $177.50', 'Required', 'RobStride power board'),
            ('res', 'Precharge and discharge resistor', 'aluminium-clad, not chosen', "10 ohm, 200 W: the board asks for 3-10 ohm and 50 W or more and won't start without one, and it also burns what braking returns. RobStride's discharge module manual gives 10 ohm, 200 W for a humanoid's limbs. Needs a metal plate to cool it, which the model doesn't have", 1, 'each', 1, None, 'not chosen', 'Required', '10 ohm 200W aluminum resistor'),
            ('fuse', 'Main fuse and disconnect', 'not chosen', '60 V DC or more, sized from measured current. The board has an internal one-time fuse too', 1, 'each', 1, None, 'not chosen', 'Required', '100A DC circuit breaker 60V'),
            ('estop', 'Emergency stop', 'not chosen', "normally-open contact, as the board's EM_NO terminal wants. **It cuts the motor branches and the robot then falls**: use it with the hoist below. A normally-open stop doesn't trip on a broken wire", 1, 'each', 1, None, 'not chosen', 'Required', 'emergency stop button normally open'),
            ('xt', 'Actuator power plugs and sockets', 'AMASS XT30U-M and XT30UW-F', "each RS04 and RS03's line end is an XT30UW-F by the manual; the RS00 has AMASS XT30PB(2+2)-M.G.B on the case and XT30(2+2)-F.G.B on the line, power and CAN together. 21 to mate with the actuators, 3 at the board", 24, 'each', 24, None, 'not chosen', 'Required', 'AMASS XT30 connector'),
            ('gh', 'CAN plugs', 'JST GH 1.25 mm, 2-pin, and "Tany" per the manual', 'GH1.25-2PWT on the board, GH1.25 on the line. 21 to mate with the actuators, 3 at the adapter', 24, 'each', 24, None, 'not chosen', 'Required', 'GH1.25 2 pin connector'),
            ('dist', 'Power distribution: one branch to 8 actuators, in each leg', 'custom, not designed', 'see [wiring](WIRING.md). Rated at 40 A at 60 V', 3, 'each', 3, None, 'not designed', 'Required', 'XT30 parallel harness'),
            ('wp', 'Power wire', 'silicone, 8 AWG and 12 AWG', '8 AWG battery to board, 12 AWG for branches. Lengths in [wiring](WIRING.md). Gauge to be checked against measured current', 1, 'lot', 1, None, 'any hobby shop', 'Required', '12 AWG silicone wire'),
            ('wc', 'CAN wire', 'twisted pair, 120 ohm', 'about 4 m in all', 1, 'lot', 1, None, 'not chosen', 'Required', '120 ohm twisted pair CAN cable'),
            ('r120', 'CAN terminator', '120 ohm, 1/4 W', '1 per bus at the far end. The hub\'s own end to be measured', 3, 'each', 3, None, 'any', 'Required', '120 ohm resistor 1/4W'),
        ]),
        ('Consumables', [
            ('loc', 'Thread locker', 'medium strength', 'every screw into an actuator, with care: confirm with RobStride it is allowed in its threads', 1, 'bottle', 1, None, 'any hardware shop', 'Required', 'medium strength thread locker'),
            ('fix', 'Cable ties, spiral wrap, heat-shrink, solder', 'generic', '', 1, 'lot', 1, None, 'any hardware shop', 'Required', 'cable ties spiral wrap heat shrink'),
        ]),
        ('Support equipment (not in the robot)', [
            ('hoist', 'Overhead hoist or gantry, and harness', 'not chosen', 'for every power-on. Rated for several times 100 kg, by someone qualified to say how many', 1, 'each', 1, None, 'not chosen', 'Required', 'overhead gantry hoist'),
            ('chg', 'Charger', 'not chosen', '13S, 54.6 V, matched to the pack and BMS', 1, 'each', 1, None, 'with the pack', 'Required', '54.6V 13S lithium charger'),
        ]),
    ]
    num, k = {}, 0
    for _, rows in GROUPS:
        for r in rows:
            k += 1
            num[r[0]] = k

    def money(v):
        return f'${v:,.2f}'

    tot, opt, unpriced, summary, T = 0.0, 0.0, 0, [], []
    for g, rows in GROUPS:
        T += [f'### {g}', '', '| Item | Part | Maker / part number | Specification | Qty per robot | Order unit | Order qty | Unit price | Line price | Source |', '|---|---|---|---|---|---|---|---|---|---|']
        sub = 0.0
        for key, part, mpn, spec_, q, unit, oq, price, src, st, words in rows:
            line = None if price is None else oq * price
            o = st == 'Optional'
            if not o:
                if line is not None:
                    sub += line
                else:
                    unpriced += 1
            elif line is not None:
                opt += line
            search = f"[Search Amazon](https://www.amazon.com/s?k={words.replace(' ', '+')})"
            T.append(f"| {num[key]} | {'**OPTIONAL:** ' if o else ''}{part} | {mpn} | {spec_} | {(f'({q})' if o else q) if q else '-'} | {unit} | {(f'({oq})' if o else oq) if oq else '-'} | {money(price) if price else '-'} | {money(line) if line else '-'} | {src}<br>{search} |")
        T.append('')
        tot += sub
        summary.append(f'| {g} | {money(sub) if sub else "-"} |')
    MATN = {'al7075': '7075-T6', 'al6061': '6061-T6', 'steel': 'steel', 'petg': 'PETG', 'tpu': 'TPU'}
    M, j = [], 0
    for r in metal:
        j += 1
        d = r['dims']
        M.append(f"| F{j} | {r['name']} | {MATN[r['mat']]} | {d[2]:.0f} x {d[1]:.0f} x {d[0]:.1f} mm | {process(r)} | {'-' if r['fill'] >= 1 else f'{r_pct(r)} of solid, assumed'} | {r['kg'] * 1000:.0f} g | {r['q']} |")
    P_, j = [], 0
    for r in printed:
        j += 1
        d = r['dims']
        P_.append(f"| P{j} | {r['name']} | {MATN[r['mat']]} | {d[2]:.0f} x {d[1]:.0f} x {d[0]:.0f} mm | {r['kg'] * 1000:.0f} g | {r['q']} |")
    n_metal = sum(r['q'] for r in metal)
    n_print = sum(r['q'] for r in printed)
    blocks = sum(1 for r in metal if process(r).startswith('milled'))
    thick = collections.Counter(round(r['dims'][0], 1) for r in metal if process(r).startswith('cut'))
    area = collections.Counter()
    for r in metal:
        if process(r).startswith('cut'):
            area[(r['mat'], round(r['dims'][0], 1))] += r['q'] * r['dims'][1] * r['dims'][2] / 1e6
    sheets = '; '.join(f"{MATN[m]} {t:g} mm, {a:.2f} m2" for (m, t), a in sorted(area.items()))
    L = ['# Big duck: bill of materials', '', UNSAFE, '',
         '| | |', '|---|---|',
         f'| Product | Big duck, biped, {sum(act.values())} actuators on 15 joints |',
         f'| Height, mass | 1.797 m, {total:.1f} kg as modelled, leaning on pocketing nobody has drawn |',
         '| Build quantity | Quantities are for 1 robot |',
         f'| Fabricated parts | {n_metal} metal, {n_print} printed: {by["al7075"]:.1f} kg 7075, {by["al6061"]:.1f} kg 6061, {by["steel"]:.1f} kg steel, {by["petg"]:.1f} kg PETG, {by["tpu"]:.1f} kg TPU. No drawing or print file exists for any of them |',
         '| Status | Design study: no part made, no actuator tested, not safe to build |',
         '| Prices | USD, as listed or found 2026-11-05, before tax and shipping. Confirm before ordering |',
         '| Related | [Wiring](WIRING.md), [its spec](../../spec/big_duck.py), [Fusion trial](FUSION.md) |', '',
         'Generated by `tools/big_docs.py` from the spec. Quantities of actuators, bearings and fabricated parts match the model. Screws follow the holes in the actuators, as the model only shows some of them.', '',
         '## Cost summary', '', '| Group | Required parts, priced |', '|---|---|', *summary,
         f'| **Total of quoted lines** | **{money(tot)}** |', '',
         f'That is a floor, not a price: {unpriced} required lines have no quote and are not in it, including the battery, its BMS, the metal, the machining and the hoist. Shipping isn\'t in it either. The optional line adds {money(opt)}.', '',
         '## Purchased parts', '', 'Every line is required unless it starts with **OPTIONAL**. Optional lines show their quantities in brackets and are not in the total. The Search Amazon links are searches only: nobody has checked what they find.', '', *T,
         '## Battery options', '', 'Read 2026-11-05. A candidate to test on a bench, with nothing bought. All are 13S, 54.6 V full, under the RS04\'s 60 V. The need: the power board trips at 80 A for 30 ms, and its 3 branches peak at 40 A each. Real current is unmeasured.', '',
         '| Pack | Nominal | Wh | Continuous, peak | Mass | Size, mm | Price | What would rule it out |', '|---|---|---|---|---|---|---|---|',
         f"| {link('Unitree G1 pack', 'https://www.roboticscenter.ai/store/product/unitree-g1-battery')} | 46.8 V, 9 Ah | 421 | not published | 2.55 kg | 120 x 80 x 182, by resellers | $750-$1,150, backorder; charger about $165-$195 | no discharge rating, and a BMS interface nobody outside publishes. A third of the energy: 80 A would be about 9C. Its BMS may refuse to run without a Unitree robot |",
         f"| **{link('Dan-Tech 13S5P soft pack', 'https://shop.danenergy.com/en-il/products/li-ion-battery-softpack-13s5p-48v-25ah-100a')}: the candidate** | 48 V, 25 Ah | 1200 | 100 A, 180 A; ratings stated without a BMS | 5.2 kg | 280 x 80 x 130 | 814 EUR by a snippet, unconfirmed; out of stock | no BMS or case, so one has to be found that keeps up. Out of stock. Soft pack: no impact protection at all |",
         f"| {link('LiTech 13S5P', 'https://litechpower.com/?p=5177')} | 48 V, 25 Ah | about 1200 | 50 A, 100 A for 3 s | about 6 kg | 300 x 125 x 95 | not read; page wouldn't open, from a search result | has a BMS, the fallback. But 50 A continuous is under the board\'s 80 A, so its BMS could cut out first and drop the robot |",
         f"| {link('CM Batteries 13S7P', 'https://cmbatteries.com/?p=23091')} | 48.1 V, 21 Ah | about 1010 | 31.5 A, 63 A for 5 s | 4.4 kg | 259 x 70 x 149 | not listed | under one leg\'s 40 A peak twice over. Made for small inspection robots |",
         f"| {link('EM3ev King Shark 13S4P', 'https://em3ev.com/batteries-48v/')}, e-bike | 48 V, 19.6 Ah | about 917 | not listed | not listed | not listed | $379 | no rating, size or mass published. An e-bike pack\'s BMS isn\'t built to take back braking current from 19 actuators |",
         '', 'Why Dan-Tech to test: it is the only one here with a published rating above the board\'s 80 A limit, so the board should trip before the pack, and it fits the body across the posts with room for a BMS. It costs safety: without a BMS it has no protection at all, and a soft pack in a falling robot could catch fire. The LiTech is simpler if measured current stays well under 50 A. Measure one RS04 on a current-limited lab supply before any pack goes near one.', '',
         '## Fabricated parts', '', 'No file for any of these exists yet: the sizes below are the spec\'s envelopes, not drawings. Left and right legs are the same parts mirrored.', '',
         '### Metal', '', f'Sheet stock to cut, as outlines only, before nesting or scrap: {sheets}. The first idea in the spec, every metal part from one sheet thickness, does not hold: sheet thicknesses are {", ".join(f"{t:g}" for t in sorted(thick))} mm, and {blocks} parts are blocks standing in for machined parts.', '',
         '| Item | Part | Material | Envelope | How | Pocketing | Mass each | Qty per robot |', '|---|---|---|---|---|---|---|---|', *M, '',
         '### Printed', '', f'Panels within a 256 mm bed by `tools/checks.py`. Masses are 3 mm walls in the shape; the panels carry no load, and have no bosses or fixings.', '',
         '| Item | Part | Material | Envelope | Mass each | Qty per robot |', '|---|---|---|---|---|---|', *P_, '',
         '## Open before ordering', '',
         'All of these stay open. Nothing here has been settled by a test or by anyone qualified to sign it off.', '',
         '### Could hurt someone', '',
         '- **No stress analysis.** Plates, shafts, pins, rods and screws have a hand estimate at most. Rods carry about 2 kN mid stroke and 3-4 kN at the ends on 10 mm pins, in 5 mm plate, on 14 x 10 bronze bushes held on by E-clips: none of it sized. The hip yaw beam, hip yoke and shin take the whole robot.',
         '- **Metal frames unchecked.** No FEA, fatigue check, weld or joint design. Many frames are solid blocks with an assumed fill, so their real strength and mass are both unknown: pocketing them may take away what they need, and leaving them solid adds mass.',
         "- **No real actuator has been measured.** RobStride's STEP files have now been read in Fusion (see below), and the hole circles and steps in the spec come from them. A file isn't a part: measure a real one before cutting metal.",
         '- **No brakes.** RobStride\'s manual describes damping when back-driven with the power off, not a brake. Power loss, a tripped branch, the emergency stop or a BMS cut-off all drop 100 kg. A fault in a branch is one the board can\'t recover from without restarting. So a hoist for every power-on, and nobody under or beside it.',
         '- **How it falls.** Not studied. Nothing protects the frames, the head, the battery or people nearby. A 1.2 kWh lithium pack in a falling robot needs its own case and impact design.',
         '- **Battery.** A candidate only, and not tested. It has no BMS: one has to take back braking current and allow the peaks, and if it cuts out, the robot drops. Fire safety for 1.2 kWh of bare soft pack in a machine that falls isn\'t designed.',
         '- **Pinch points.** Cranks, rods, the chain, and paired actuators can take a hand off. No guards are designed.',
         '- **Paired actuators.** Hips and knees share one axis between two actuators. If one faults, or their zeros differ, they fight each other through the frame. No startup check or fault response is written.',
         '- **Chain.** The tensioner, guards, fatigue and what happens if it breaks: the head and its yaw and roll, about 14 kg, drop. Not designed.',
         '',
         '### The real actuators against the model', '',
         "- **Shape: read from the STEP files, 2026-11-05.** The RS04 is 120 across for 39 mm and 94 across for its rear 13.2, as the manual's figure says; 55.7 long pin tip to tail, 52.7 from output face to tail; the output face stands 0.5 past the stator's. The RS03 isn't a cylinder: 92.8 across with 8 lobes to 106.6, a rear step 79.5 across and 14.6 long, 56.6 long. The RS00 is 57 x 52.1. So every stator now hangs by its shoulder, through a plate bored for the step. The plates on rear faces were an error: there's no flat rear face to bolt to. No file holds a mass, so the 1.42 kg stays the retailer's.",
         "- **Cable.** Each RS04's leaves in a 7.3 mm sleeve, radially, 6.3 mm behind the shoulder, ending at r 58.1: each plate has a 9 mm window for it, closed at r 64, between two screws. The cable then lies on the plate's far side. Routes are sketched in [wiring](WIRING.md), not drawn: nobody has shown the 310 mm reaches, and the cable's real bend radius is unread. The RS03's plugs are on the sides of its rear step, past its plate. Not designed: strain relief, guards, and the moving loops across each joint.",
         "- **Pins.** 3 x 6 mm on 35.86, standing 3.0 mm in the file, not the manual's 3.3. Every flange now has holes for them, 3.5 deep. The RS03 has 3 x 4 mm holes, 7 deep, not pins: its flanges want bought dowels, modelled as holes only. They may be what carries the torque: ask RobStride what the screws and pins are each rated for.",
         "- **Screws: the thread is the problem.** The file's RS04 output holes are blind at 3.5 mm, where the manual says 6.5 (6.5 is the file's distance from that floor to the pin tips, so the figure may be read wrongly). Every output screw is now sized to engage 3 mm, so none bottoms. That's under 4 turns of M5, on 9 screws. **Measure a real one**: if 3.5 is right, the screws may hold little, and the pins everything. Lengths by joint are in the spec and [the Fusion notes](FUSION.md).",
         "- **Frame joints: designed for fit, not strength (2026-11-05).** Every pair of metal parts that touch now shares a screw, or a named fit, key or strap (`HELD` in `spec/big_duck.py`), and an assembly order exists that `tools/big_assembly.py` checks: each part's way in, each screw's straight path and key, thread engagement at least 1.25 x the diameter, metal round every hole. See [ASSEMBLY.md](ASSEMBLY.md). What that leaves: many joints are M4 tapped into the edge of 8 mm plate (2.4 mm of wall each side), the head pitch stator plate hangs from one tie, the head roll cradle from 3 M4 into its edge, and the hip roll cheeks from 5 M5 each. Nobody has checked any of them carries its load. Fits (H7, N7, k6, r6) and keys are named, not drawn. The tools for printed parts still don't run on this duck: `buildable`, `export_prints`, `printable`, `beak_linkage`.",
         "- **Panels have no fixings.** The body, hood, face, bill, beak and shin panels touch the frame, and some cut into it, with nothing designed to hold them: the assembly check lists them as warnings. Only the thigh covers and the shoes are screwed on.",
         '- **Torque and heat.** The 40 N m rating needs the RS04 on a 345 x 345 mm aluminium plate; 35 on 220 x 200. These frames are neither, so true ratings are lower and unknown. The manual\'s overload table, on that large plate: 120 N m for 3 s, 100 for 15 s, 80 for 38 s, 60 for 108 s, 50 for 324 s. So the "240 N m" pair is a 3-second number. A crouch asks each knee RS04 for about 34 N m, and the first walk for about 55, RMS: minutes at best. The manual\'s winding limit is 145 C.',
         '- **Numbers disagree.** RobStride\'s selection table says the RS04 runs on 15-60 V, its manual 24-60. The retailer gives 1.5 kg and 1420 g on one page.',
         '- **The beak.** Its rod and crank aren\'t designed. Nor is the head\'s contents: 0.6 kg of "head boards and camera" is a placeholder. The RS00\'s real connector is on its side.',
         '',
         '### Power and control', '',
         '- **Current has never been measured or simulated.** Each leg gets one 20 A rated, 40 A peak branch, 1.9 kW at 48 V peak, for 8 RS04s of 700 W rated each. It may need more branches or a second board. Exceeding it stops the leg until a restart, and the robot falls.',
         '- **Regenerated power.** Set the board\'s discharge point (parameter 0x8003) with care. Its manual has discharge start at this value plus 6 V, and the actuators\' maximum is 60 V, so with a 54.6 V pack it should read 54, not its default 58. Nothing has tested this.',
         '- **Layout.** The model\'s single box, 90 x 170 x 30 mm, stands for the computer and board and shows no connectors. On the real board the battery, resistor and rail connectors face one edge and the branches the opposite, and the gap to the battery is 10 mm. The boards have to be modelled, and it probably won\'t fit as it is.',
         f'- **CAN.** Stubs are 350 mm by the harness, over the 300 mm usually allowed at 1 Mbps: test for errors or cut them. Whether each actuator has its own terminator is unclear: RobStride\'s firmware notes mention one on an older RS03. Measure each bus between H and L with the power off.',
         '- **Sources.** All actuators are on backorder at the shop read, and no second US source was confirmed. Items with "not chosen" have no source. The CAN hub\'s page wouldn\'t open.',
         '- **Software.** The Pi code is for the small ducks\' serial servos. RobStride has its own CAN protocol, unwritten here.',
         '- **Amazon links.** Only the 2 parts shared with the all-Dynamixel duck have product links, as last read for it. Every other is a search nobody has followed. Confirm before ordering.',
         '- **Not in this design:** the walk\'s own limits (it overloads its knee at 0.33 m/s; see [the notes](../DETAILS.md#a-big-duck)), speaker, microphone.', '']
    folder = G.out_path('docs')
    os.makedirs(folder, exist_ok=True)
    open(os.path.join(folder, 'BOM.md'), 'w', encoding='utf-8', newline='\n').write('\n'.join(L))
    return tot, unpriced, n_metal, n_print


def r_pct(r):
    return f"{r['fill'] * 100:.0f} %"


if __name__ == '__main__':
    print(bom_big(G.load()))
