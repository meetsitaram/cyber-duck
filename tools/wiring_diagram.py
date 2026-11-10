"""Draws the wiring from spec/wiring_dynamixel.json, and checks it, to docs/dynamixel/wiring.png. Change the config, not the picture.

  python -I tools/wiring_diagram.py

Part pictures are in docs/dynamixel/parts/, named in the config.
"""
import json
import os
import sys

from PIL import Image, ImageChops, ImageDraw, ImageFont

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PARTS = os.path.join(ROOT, 'docs', 'dynamixel', 'parts')
W, H = 1760, 1250
INK, GREY = (24, 28, 30), (90, 96, 100)


def font(s, bold=False):
    return ImageFont.truetype('arialbd.ttf' if bold else 'arial.ttf', s)


def tidy():
    """Fusion's raw renders, cropped to the part."""
    for f in os.listdir(PARTS):
        if f.endswith('_raw.png'):
            im = Image.open(os.path.join(PARTS, f)).convert('RGB')
            box = ImageChops.difference(im, Image.new('RGB', im.size, im.getpixel((0, 0)))).point(lambda v: 255 if v > 18 else 0).getbbox()
            im.crop(box).save(os.path.join(PARTS, f.replace('_raw', '')))
            os.remove(os.path.join(PARTS, f))


def check(c):
    """Every wire joins real pins of the same kind's route; every pin is used."""
    bad, used = [], set()
    for w in c['wires']:
        for end in (w['from'], w['to']):
            part, _, pin = end.partition('.')
            if part not in c['parts'] or pin not in c['parts'][part]['pins']:
                bad.append(f'{end}: no such pin')
            used.add(end)
        if w['kind'] not in c['kinds']:
            bad.append(f"{w['from']}: no such wire kind {w['kind']}")
    for n, p in c['parts'].items():
        for pin in p['pins']:
            if f'{n}.{pin}' not in used:
                bad.append(f'{n}.{pin}: not connected')
    return bad


def main():
    global PARTS
    # (a private version: a config, a folder of pictures and an output path, as arguments)
    cfg, out = (sys.argv[1], sys.argv[3]) if len(sys.argv) > 3 else (os.path.join(ROOT, 'spec', 'wiring_dynamixel.json'), os.path.join(ROOT, 'docs', 'dynamixel', 'wiring.png'))
    if len(sys.argv) > 3:
        PARTS = sys.argv[2]
    c = json.load(open(cfg, encoding='utf-8'))
    for k in c['kinds'].values():
        k['color'] = tuple(k['color'])
    if len(sys.argv) <= 3:
        tidy()
    bad = check(c)
    print(f"{len(c['parts'])} parts, {len(c['wires'])} wires, {len(bad)} failing")
    for b in bad:
        print(' FAIL', b)
    if bad:
        sys.exit(1)
    W_, H_ = c.get('size', (W, H))
    im = Image.new('RGB', (W_, H_), 'white')
    d = ImageDraw.Draw(im)
    for x0, y0, x1, y1, t in c.get('bands', ()):        # where the parts are: what crosses between 2 bands goes up the neck
        d.rounded_rectangle((x0, y0, x1, y1), 18, outline=(190, 194, 198), width=2)
        d.text((x1 - 20, y0 + 22), t, fill=GREY, font=font(26, True), anchor='rm')
    d.text((60, 36), c['title'], fill=INK, font=font(46, True))
    at = {}
    boxes = {}
    # pictures first, so wires end at their edges
    for n, p in c['parts'].items():
        if p['picture'] == 'dot':
            boxes[n] = (p['x'], p['y'], 0, 0)
        elif p['picture']:
            pic = Image.open(os.path.join(PARTS, p['picture'])).convert('RGB')
            h = int(p['w'] * pic.height / pic.width)
            im.paste(pic.resize((p['w'], h), Image.LANCZOS), (p['x'], p['y']))
            boxes[n] = (p['x'], p['y'], p['w'], h)
        else:
            d.rounded_rectangle((p['x'], p['y'], p['x'] + p['w'], p['y'] + p['h']), 14, outline=c['kinds']['charge']['color'], width=4)
            boxes[n] = (p['x'], p['y'], p['w'], p['h'])
        x, y, w, h = boxes[n]
        for pin, (u, v) in p['pins'].items():
            at[f'{n}.{pin}'] = (x + u * w, y + v * h)
    labels = []
    for wire in c['wires']:
        k = c['kinds'][wire['kind']]
        a, b = at[wire['from']], at[wire['to']]
        pts = [a, *[tuple(q) for q in wire.get('via', [])]]
        # straight runs only: step to the end point's height or line, last
        last = pts[-1]
        if last[0] != b[0] and last[1] != b[1]:
            pts.append((last[0], b[1]) if wire.get('via') else ((last[0] + b[0]) / 2, last[1]))
            if not wire.get('via'):
                pts.append((pts[-1][0], b[1]))
        pts.append(b)
        d.line(pts, fill=k['color'], width=k['width'], joint='curve')
        for q in (a, b):
            r = k['width'] * 0.8
            d.ellipse((q[0] - r, q[1] - r, q[0] + r, q[1] + r), fill=k['color'])
        if wire.get('label'):
            seg = max(zip(pts, pts[1:]), key=lambda s: abs(s[1][0] - s[0][0]) + abs(s[1][1] - s[0][1]))
            flat = abs(seg[1][1] - seg[0][1]) < 2   # along a level wire the label sits above it, so the wire stays visible
            labels.append((*wire.get('label_at', ((seg[0][0] + seg[1][0]) / 2, (seg[0][1] + seg[1][1]) / 2 - (26 if flat else 0))), wire['label'], k['color']))
    for x, y, t, col in labels:
        f = font(24, True)
        tw = d.textlength(t, font=f)
        d.rounded_rectangle((x - tw / 2 - 10, y - 19, x + tw / 2 + 10, y + 19), 8, fill='white')
        d.text((x, y), t, fill=col, font=f, anchor='mm')
    for n, p in c['parts'].items():
        if not p['label']:
            continue
        x, y, w, h = boxes[n]
        if p['picture'] is None:
            d.text((x + 24, y + 28), p['label'], fill=INK, font=font(29, True), anchor='lm')
            d.text((x + 24, y + 62), p['sub'], fill=GREY, font=font(23), anchor='lm')
        else:
            top = p.get('label_at') == 'top'
            d.text((x, y - 66 if top else y + h + 26), p['label'], fill=INK, font=font(31, True), anchor='lm')
            d.text((x, y - 30 if top else y + h + 62), p['sub'], fill=GREY, font=font(24), anchor='lm')
    x = 60
    for k in c['kinds'].values():
        d.line((x, H_ - 118, x + 50, H_ - 118), fill=k['color'], width=k['width'])
        d.text((x + 62, H_ - 118), k['name'], fill=INK, font=font(23), anchor='lm')
        x += 104 + d.textlength(k['name'], font=font(23))
    for i, t in enumerate(c['footer']):
        d.text((60, H_ - 76 + 32 * i), t, fill=GREY, font=font(24), anchor='lm')
    im.save(out)
    print(out)


if __name__ == '__main__':
    main()
