"""The Feetech HD-1910 duck's bill of materials, written for someone building it to order: docs/hd1910/BOM.md. Called by tools/make_docs.py.

Shared parts keep the all-Dynamixel duck's Amazon links, as last read there; no Amazon page was opened for this list. New lines have a search link only, and are listed under "Open before ordering".
"""
import collections
import json
import os

SHEET = 'FEETECH spec sheet A/0, 2026-09-07'


def horn_screws(spec):
    """Machine M2 screws, by length: whatever goes into a servo's horn. The rest of the M2s are self-tapping, into plastic."""
    c = collections.Counter()
    for l in spec['links'].values():
        for g in l['geoms']:
            if 'fast' in g and ' fork screw ' in g['name'] and g['fast']['size'] == 'M2':
                c[round(g['fast']['L'] * 1000)] += 1
    return c


def write(spec, counts, MM, G, amz):
    servos, bear, screws = counts(spec)
    inert = MM.link_inertials(spec)
    ps = [p for v in inert.values() for p in v[3]]
    pla, tpu = sum(p[2] for p in ps if p[1] == 'pla'), sum(p[2] for p in ps if p[1] == 'tpu')
    total = sum(v[0] for v in inert.values())
    n = servos['HD1910_C001']
    machine = horn_screws(spec)
    tap6, tap8 = screws[('M2', 6)] - machine[6], screws[('M2', 8)] - machine[8]
    assert screws[('M2', 5)] == machine[5], 'an M2 x 5 that is not a horn screw'
    tall = sum(screws[k] for k in screws if k[0] == 'M3')

    def link(label, url):
        return f'[{label}]({url})'

    # (key, part, maker and part number, specification, quantity per robot, what is ordered, how many, price each or None, source, Required / Optional)
    GROUPS = [
        ('Actuators', [
            ('servo', 'Smart servo, serial bus', 'Feetech HD-1910-C001', f'15 kg cm stall (+/- 10 %), 2 A and 113 rev/min at 7.4 V; 3.7 kg cm rated; 4-8.4 V; 1:320; 21 g; 34 x 20 x 23 mm, 29 mm across its 2 horns; 12-bit magnetic encoder; AMP2.0-3P sockets; 15 cm lead', n, 'each', n, 34.00,
             link('AiFitLab', 'https://aifitlab.com/products/feetech-hd1910-servo-motor') + ': backordered on 2026-11-05. Second source ' + link('OpenELAB', 'https://openelab.io/products/feetech-hd1910-microduck-coreless-servo') + ' at EUR 33.55, sold out on 2026-11-05', 'Required'),
        ]),
        ('Bearings', [
            ('mr2216', 'Ball bearing, around each horn', 'MR2216ZZ (ET2216ZZ)', '16 x 22 x 4 mm', bear['MR2216ZZ'], 'pack of 10', 2, 42.72, amz('B0C5ZCVGNP'), 'Required'),
            ('b6700', 'Ball bearing, shoe inner ear', '6700ZZ', '10 x 15 x 4 mm', bear['6700ZZ'], 'pack of 10', 1, 6.79, amz('B0FQ4VYXRP'), 'Required'),
        ]),
        ('Fasteners', [
            ('hm5', 'Screw, into a horn, machine thread', 'generic', 'M2 x 5 mm, pan head, Phillips', machine[5], 'pack of 50', 1, None, 'search', 'Required'),
            ('hm6', 'Screw, into a horn, machine thread', 'generic', 'M2 x 6 mm, pan head, Phillips', machine[6], 'pack of 50', 1, None, 'search', 'Required'),
            ('hm8', 'Screw, into a horn, machine thread', 'generic', 'M2 x 8 mm, pan head, Phillips', machine[8], 'pack of 50', 1, None, 'search', 'Required'),
            ('t6', 'Screw, into printed parts, self-tapping', 'generic', 'M2 x 6 mm, pan head', tap6, 'in item below', 0, None, 'from the M2 kit below', 'Required'),
            ('t8', 'Screw, servo frame holes and printed parts, self-tapping', 'generic', 'M2 x 8 mm, pan head', tap8, 'in item below', 0, None, 'from the M2 kit below', 'Required'),
            ('m2kit', 'Self-tapping M2 screws', 'generic', 'pan head kit, 800 pieces', tap6 + tap8, 'kit', 1, 15.66, amz('B0GV338JJK'), 'Required'),
            ('m2f', 'Screw, flat head, machine thread: ankle rear horns', 'generic', 'M2 x 4 mm, Phillips', screws[('M2 flat', 4)], 'pack of 50', 1, None, 'search', 'Required'),
            ('m25', 'Screw, bus board, machine thread', 'generic', 'M2.5 x 5 mm, pan head, Phillips, black oxide 304 stainless', screws[('M2.5', 5)], 'pack of 50', 1, None, 'Amazon', 'Required'),
            ('so', 'Spare hex standoffs, bus board: 4 come with the Waveshare board', 'uxcell', 'brass, M2.5, female-female, 8 mm long, 5 mm across flats', 0, 'pack of 50', 1, None, 'Amazon', 'Optional'),
            ('m3kit', 'Screws, socket cap: ' + ', '.join(f'{screws[k]} x M3 x {k[1]}' for k in sorted(screws) if k[0] == 'M3'), 'generic', 'kit of 610 with nuts and washers', tall, 'kit', 1, 19.99, amz('B09NR8X2LV'), 'Required'),
            ('wash', 'Washer, far hinge tab to hood', 'uxcell', 'M3, 6 mm outside, 0.5 mm thick, stainless', 1, 'pack of 100', 1, None, amz('B015A39NCC'), 'Required'),
        ]),
        ('Electronics', [
            ('pi', 'Computer', 'Raspberry Pi Zero 2 W', 'without header', 1, 'each', 1, None, link('PiShop', 'https://www.pishop.us/product/raspberry-pi-zero-2-w/') + ': out of stock on 2026-11-03', 'Required'),
            ('sd', 'microSD card', 'SanDisk Ultra', '32 GB microSDHC, A1, Class 10, UHS-I', 1, 'each', 1, 22.49, amz('B08L5HMJVW'), 'Required'),
            ('board', 'Servo bus board', 'Waveshare Bus Servo Adapter (A), SKU 25514', 'USB-C, 2 servo sockets at 2.5 mm pitch, 42 x 33 mm, also sold as Serial Bus Servo Driver Board', 1, 'each', 1, 4.99, link('Waveshare', 'https://www.waveshare.com/bus-servo-adapter-a.htm') + '; ' + amz('B0CTMM4LWK') + '. Second source ' + link('Abra', 'https://abra-electronics.com/electromechanical/motors/motor-controllers/wave-25514-serial-bus-servo-driver-board.html') + ' at $9.05', 'Required'),
            ('reg', '5 V regulator', 'hiBCTR USB buck converter QC3.0', '6-32 V in, 35 x 17 x 8.5 mm', 1, 'pack of 2', 1, 7.97, 'Amazon', 'Required'),
            ('imu', 'IMU', 'BNO085 module, EC Buying', 'modelled at 25.6 x 22.7 x 4.6 mm; I2C', 1, 'each', 1, 20.49, amz('B0CDGZMLPP') + '. Second source ' + link('Adafruit 4754', 'https://www.adafruit.com/product/4754') + ' at $29.50, in stock on 2026-11-03', 'Required'),
            ('cam', 'Camera', 'Arducam 8 MP IMX219', 'board modelled at 24 x 24 mm', 1, 'each', 1, 16.99, amz('B09V576TFN'), 'Optional'),
            ('ribbon', 'Camera ribbon, if the camera lacks one', 'GODIYMODULES', 'Pi Zero 22-pin to 15-pin, 15 cm', 1, 'pack of 2', 1, None, amz('B0F9WJ62LV'), 'Optional'),
        ]),
        ('Power and wiring', [
            ('batt', 'Battery', 'OVONIC 2S LiPo, 100C', '450 mAh, XT30, 61.5 x 15.8 x 13.5 mm, 31 g', 1, 'pack of 4', 1, 26.39, amz('B0D2KT723L'), 'Required'),
            ('xt30', 'Battery connector', 'Nuofany XT30 pigtail', '100 mm, 16 AWG', 1, '5 pairs', 1, 9.99, amz('B0B4H4GKQL'), 'Required'),
            ('wire', 'Power wire, red and black', 'BNTECHGO silicone wire', '20 AWG', 1, '10 ft each', 1, None, amz('B01M12PSJM'), 'Required'),
            ('thin', 'Signal wire: IMU', 'Fermerry silicone wire', '28 AWG stranded, 4 wires, about 1 m in all', 1, '6 colours, 5 ft each', 1, None, amz('B089CR4SDM'), 'Required'),
            ('bus', 'Bus wire: 2 long servo leads, spliced', 'silicone wire', '22 AWG, 3 colours, about 1 m in all', 1, 'lot', 1, None, 'search', 'Required'),
            ('plug', 'Board plug, on a lead: into the bus board\'s 2 sockets', 'generic, as an STS3215 lead', '3-pin plug, 2.5 mm pitch, type 5264, one end cut off', 2, 'each', 2, None, 'search', 'Required'),
            ('usb', 'Regulator to Pi power', 'LY tech flat ribbon USB', 'right-angle USB-A to right-angle micro USB, 10 cm', 1, 'each', 1, 13.50, 'Amazon', 'Required'),
            ('otg', 'Pi to bus board', 'Veeloon', 'right-angle micro USB OTG to right-angle USB-C, 0.6 ft', 1, 'pack of 2', 1, 7.99, amz('B0DKBQ7TD3'), 'Required'),
        ]),
        ('Consumables', [
            ('fix', 'Foam tape, hook-and-loop strap, small cable ties', 'generic', '', 1, 'lot', 1, 12.00, 'any hardware shop; price estimated', 'Required'),
            ('joint', 'Solder, heat-shrink tube, CA glue', 'generic', '', 1, 'lot', 1, None, 'any hardware shop', 'Required'),
        ]),
        ('Support equipment (not in the robot)', [
            ('chg', 'Charger', 'as linked', '2S LiPo, to 8.4 V, must match the balance plug', 1, 'each', 1, None, amz('B0FDKX4V38'), 'Required'),
        ]),
    ]
    num, k = {}, 0
    for _, rows in GROUPS:
        for r in rows:
            k += 1
            num[r[0]] = k
    SEARCH = {'servo': 'Feetech HD-1910-C001 servo', 'mr2216': 'MR2216ZZ bearing 16x22x4mm', 'b6700': '6700ZZ bearing 10x15x4mm', 'hm5': 'M2 x 5mm pan head phillips machine screw', 'hm6': 'M2 x 6mm pan head phillips machine screw', 'hm8': 'M2 x 8mm pan head phillips machine screw',
              't6': 'M2 x 6mm self tapping pan head screw', 't8': 'M2 x 8mm self tapping pan head screw', 'm2kit': 'M2 self tapping pan head screw kit', 'm2f': 'M2 x 4mm flat head phillips machine screw', 'm25': 'M2.5 x 5mm pan head machine screw black oxide',
              'so': 'uxcell M2.5 x 8mm female female hex brass standoff', 'm3kit': 'M3 socket cap screw kit', 'wash': 'uxcell M3 x 6mm x 0.5mm flat washer',
              'pi': 'Raspberry Pi Zero 2 W', 'sd': 'SanDisk 32GB Ultra microSDHC A1', 'board': 'Waveshare Serial Bus Servo Driver Board', 'reg': 'hiBCTR USB buck converter QC3.0', 'imu': 'BNO085 IMU module',
              'cam': 'Arducam 8MP IMX219 camera module', 'ribbon': 'Raspberry Pi Zero camera cable 22 pin to 15 pin 15cm', 'batt': 'OVONIC 2S 450mAh 100C XT30', 'xt30': 'XT30 pigtail 16 AWG 100mm', 'wire': '20 AWG silicone wire red black',
              'thin': '28 AWG silicone stranded wire', 'bus': '22 AWG silicone wire 3 colours', 'plug': '5264 3 pin connector cable 2.5mm pitch', 'usb': 'LY tech flat ribbon micro USB 90 degree', 'otg': 'micro USB to USB C OTG cable 90 degree',
              'fix': 'foam tape hook and loop strap cable ties', 'joint': 'solder heat shrink CA glue', 'chg': '2S LiPo balance charger JST-XH'}

    def search(key):
        return f"[Search Amazon](https://www.amazon.com/s?k={SEARCH[key].replace(' ', '+')})"

    def money(v):
        return f'${v:,.2f}'
    tot, opt, unpriced, summary, T = 0.0, 0.0, 0, [], []
    for g, rows in GROUPS:
        T += [f'### {g}', '', '| Item | Part | Maker / part number | Specification | Qty per robot | Order unit | Order qty | Unit price | Line price | Source |', '|---|---|---|---|---|---|---|---|---|---|']
        sub = 0.0
        for key, part, mpn, spec_, q, unit, oq, price, src, st in rows:
            line = None if price is None else oq * price
            if st == 'Required':
                if line is not None:
                    sub += line
                elif oq:
                    unpriced += 1
            elif line is not None:
                opt += line
            o = st == 'Optional'
            T.append(f"| {num[key]} | {'**OPTIONAL:** ' if o else ''}{part} | {mpn} | {spec_} | {(f'({q})' if o else q) if q else '-'} | {unit} | {(f'({oq})' if o else oq) if oq else '-'} | {money(price) if price else '-'} | {money(line) if line else '-'} | {src}<br>{search(key)} |")
        T.append('')
        tot += sub
        summary.append(f'| {g} | {money(sub) if sub else "-"} |')
    parts = json.load(open(G.parts_path(), encoding='utf-8'))['parts']
    COL = {'#3A3D42': 'Dark grey', '#E8642A': 'Orange', '#ECEBE6': 'White', '#9A9C98': 'Mid grey', '#F0A030': 'Amber', '#0A1420': 'Black', '#F2B233': 'Yellow', '#F7AE55': 'Pale orange'}
    total_v = {m: sum(q['volume_cm3'] for q in parts.values() if q['material'] == m) for m in ('pla', 'tpu')}

    def grams(q):
        return (pla if q['material'] == 'pla' else tpu) * 1000 * q['volume_cm3'] / total_v[q['material']]
    F, vol, j = [], {}, 0
    for name, q in parts.items():
        kk = (q['material'], q['color'])
        vol[kk] = vol.get(kk, 0) + grams(q)
        if name.startswith('R '):
            continue
        j += 1
        fn = name.replace(' ', '_') + '.stl'
        two = name.startswith('L ')
        assert os.path.exists(os.path.join(G.out_path('prints'), fn)), fn
        if two:
            assert 'R ' + name[2:] in parts, name
        shown = name[2:].capitalize() + ', left and right' if two else name.capitalize()
        files = f"`{fn}`, `R_{fn[2:]}`" if two else f'`{fn}`'
        F.append(f"| P{j} | {shown} | {files} | {'PLA' if q['material'] == 'pla' else 'TPU 95A'} | {COL.get(q['color'], q['color'])} | {q['color']} | {grams(q):.1f} g | {2 if two else 1} |")
    fil = [f"| {COL.get(c, c)} | {c} | {'PLA' if m == 'pla' else 'TPU 95A'} | {g_:.0f} g | {g_ * 1.15:.0f} g |" for (m, c), g_ in sorted(vol.items(), key=lambda x: (x[0][0], -x[1]))]
    L = ['# Feetech HD-1910 duck: bill of materials', '',
         '| | |', '|---|---|',
         '| Product | Feetech HD-1910 duck, biped, 15 servos |',
         f'| Height, mass | 0.295 m, {total:.2f} kg as modelled |',
         '| Build quantity | Quantities are for 1 robot |',
         f'| Fabricated parts | {len(parts)} printed parts, {pla * 1000:.0f} g PLA and {tpu * 1000:.0f} g TPU 95A |',
         '| Status | Prototype: no unit built, no part test printed |',
         '| Prices | USD, as listed 2026-11-02 to 2026-11-06, before tax and shipping. Confirm before ordering |',
         '| Related | [Print files](../../prints/hd1910/README.md), [wiring](WIRING.md), [assembly](ASSEMBLY.md) |', '',
         'Generated by `tools/make_docs.py` from the spec. Quantities match the model.', '',
         '## Cost summary', '', '| Group | Required parts, priced |', '|---|---|', *summary,
         f'| **Total of quoted lines** | **{money(tot)}** |', '',
         f'{unpriced} required lines have no quote yet and are not in the total, nor is filament. Optional parts add {money(opt)} on the quoted lines.', '',
         '## Purchased parts', '', 'Every line is required unless it starts with **OPTIONAL**. Optional lines show their quantities in brackets and are not in the total.', '', *T,
         '## Fabricated parts', '', 'All 3D printed. Files are in [prints/hd1910/](../../prints/hd1910/README.md), already oriented for printing, in millimetres. Mass is each piece. The hood has an orange band, painted or split in the slicer.', '',
         '| Item | Part | File | Material | Colour | Hex | Mass each | Qty per robot |', '|---|---|---|---|---|---|---|---|', *F, '',
         '### Filament', '', '| Colour | Hex | Material | Model mass | With 15 % for supports and scrap |', '|---|---|---|---|---|', *fil, '',
         '## Build notes that affect ordering', '',
         f"- **Item {num['servo']}:** rated to 8.4 V, so a full 2S pack is inside Feetech's limits. Its factory settings: ID 1, 1 Mbps, position PD mode; over-current protection off. No idlers to buy: each servo has its own rear horn.",
         f"- **Items {num['hm5']}-{num['hm8']}:** Feetech draws the horn's holes as 8-M2.0, so these are machine screws. They go right through the horn: a longer one than listed drags on the case.",
         f"- **Item {num['m2kit']}:** check that it holds {tap6} M2 x 6 and {tap8} M2 x 8.",
         f"- **Item {num['board']}:** set both jumpers to B. Its silkscreen says DC 9-12.6 V and the battery is 7.4-8.4 V: bench test before ordering in quantity.",
         f"- **Items {num['plug']}, {num['bus']}:** the servos' plugs are 2.0 mm pitch and the board's sockets 2.5 mm, so no servo lead fits the board. Each servo's own lead covers its link in the chain. 2 of them are cut to make the 3 special ones: [WIRING.md](WIRING.md).",
         f"- **Item {num['imu']}:** its listing names both BNO080 and BNO085: confirm which chip arrives, its size, and that it starts in I2C mode. The model and fit come from Adafruit's board.",
         f"- **Item {num['reg']}:** feeds the Pi's PWR port only. It can output up to 12 V if a device asks for fast charging, so measure about 5 V before connecting.",
         f"- **Item {num['batt']}:** marked HV. Charge as a normal 2S, to 8.4 V.",
         f"- **Item {num['usb']}:** check the Pi's voltage under load, as the ribbon's wires are thin.", '',
         '## Open before ordering', '',
         f"- **Servo supply:** item {num['servo']} is backordered at AiFitLab and sold out at OpenELAB on 2026-11-05, and neither gives a date. {n} units is the longest lead time here. Order 1 first.",
         f"- **What's in the servo's box:** Feetech's sheet says \"No Accessories\", and its packing photo shows a lead and 8 small screws. This list assumes the lead and buys every screw: read the first box. If there's no lead, you need {n} more, and a source.",
         f"- **Sources to choose:** items {', '.join(str(num[x]) for x in ('hm5', 'hm6', 'hm8', 'm2f', 'bus', 'plug'))}: no listing has been read for them.",
         f"- **Measure on the first servo:** the plug's pitch and family against {SHEET}'s AMP2.0-3P; a horn in the bore of item {num['mr2216']}; an M2 machine screw in a horn hole, as the threads are from the drawing; the rear horn coming off cleanly. Also the Waveshare board's socket pitch (2.5 mm is measured from Waveshare's photo) and its standoffs against their printed hex sockets, 5 mm across flats (item {num['board']}); the camera board (item {num['cam']}).",
         f"- **Protocol:** Feetech's pages don't say which of its protocols the HD-1910 speaks, and the Waveshare board is sold for its ST and SC servos. Test one servo on the board before ordering {n}.",
         f"- **Amazon listings** (items {', '.join(str(num[x]) for x in ('mr2216', 'b6700', 'm2kit', 'wash', 'board', 'imu', 'otg', 'batt', 'xt30', 'wire', 'thin', 'cam'))}, and those found by search) are as read for the all-Dynamixel duck, or not read at all: confirm each page before ordering.",
         '- **Not in this design:** power switch (the XT30 is the disconnect), speaker, microphone.', '']
    folder = G.out_path('docs')
    os.makedirs(folder, exist_ok=True)
    open(os.path.join(folder, 'BOM.md'), 'w', encoding='utf-8').write('\n'.join(L))
