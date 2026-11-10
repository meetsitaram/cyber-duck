"""The servo recipes, front on and at one scale, with height, width and weight: python tools/render_recipes.py -> docs/images/recipes/recipes.png

Offline renders of each recipe's spec (spec/robot.py's RECIPES; generate the spec first, with MILLI_RECIPE set). The numbers come from the spec,
as tools/checks.py and tools/mass_model.py measure them; the dimension lines are drawn from the picture, which also checks the two agree. Plain python.
"""
import json
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, '..', 'docs', 'images', 'recipes')
RECIPES = (('dynamixel', 'All Dynamixel', ['15 x XL330-M288']),
           ('hd1910', 'Feetech HD-1910', ['15 x HD-1910-C001']),
           ('hybrid', 'Hybrid', ['8 x STS3215-C044, 2 x STS3215-C046', '1 x XC330-M288, 4 x XL330-M288']),
           ('feetech', 'All Feetech', ['10 x STS3215-C044', '5 x STS3215-C046']))
SIZE = 1400                                        # pixels; the camera's scale and framing are the same in all of them
GAP = 300                                          # between one robot's side and the next's, for the height's line and the labels
# a mid blue, as white shells and black servos both have to show against it
BG, INK, LINE, FAINT = (84, 112, 140), (255, 255, 255), (235, 240, 245), (150, 170, 190)


def child(recipe, out):
    sys.path.insert(0, HERE)
    import checks as C
    import geom as G
    import mass_model as MM
    import render_pose as RP
    spec = G.load()
    S = C.Solids(spec)
    m = C.measure(spec, S)
    json.dump(dict(height=m['height_m'], width=m['overall_width_m'], mass=sum(v[0] for v in MM.link_inertials(spec, S).values())), open(out + '.json', 'w'))
    RP.Scene().render({}, 'front', SIZE, SIZE, 0.46, 0.215, background=BG).save(out + '.png')


def main():
    import numpy as np
    from PIL import Image, ImageDraw, ImageFont
    if len(sys.argv) > 1:
        child(sys.argv[1], sys.argv[2])
        return 0
    os.makedirs(OUT, exist_ok=True)
    F = {n: ImageFont.load_default(s) for n, s in (('title', 44), ('text', 32), ('dim', 34))}
    bots = []
    with tempfile.TemporaryDirectory() as tmp:
        for r, title, servos in RECIPES:
            p = os.path.join(tmp, r)
            subprocess.run([sys.executable, os.path.abspath(__file__), r, p], check=True, env=dict(os.environ, MILLI_RECIPE=r))
            info = json.load(open(p + '.json'))
            im = Image.open(p + '.png').convert('RGB')
            a = np.asarray(im).astype(int)
            mask = (a != np.array(BG)).any(axis=2)
            ys, xs = np.where(mask)
            x0, x1, y0, y1 = xs.min(), xs.max() + 1, ys.min(), ys.max() + 1
            bots.append(dict(title=title, servos=servos, im=im.crop((x0, y0, x1, y1)), w=x1 - x0, h=y1 - y0, **info))
    scales = [b['h'] / b['height'] for b in bots] + [b['w'] / b['width'] for b in bots]
    print('pixels per metre, by height and width, each recipe:', [round(s) for s in scales])
    assert max(scales) / min(scales) < 1.06, 'the picture and the spec disagree'
    H = max(b['h'] for b in bots)
    W = sum(b['w'] for b in bots) + GAP * len(bots) + 120
    top, under = 70, 330
    C_ = Image.new('RGB', (W, top + H + under), BG)
    D = ImageDraw.Draw(C_)
    x, floor = GAP, top + H

    def arrow(p, q, label, side):
        D.line([p, q], fill=LINE, width=4)
        for c, o in ((p, q), (q, p)):
            v = np.array(o, float) - c
            v /= np.linalg.norm(v)
            n = np.array([-v[1], v[0]])
            for sg in (1, -1):
                D.line([tuple(c), tuple(np.array(c) + 22 * v + sg * 10 * n)], fill=LINE, width=4)
        if side == 'left':
            D.text((p[0] - 18, (p[1] + q[1]) / 2), label, fill=INK, font=F['dim'], anchor='rm')
        else:
            D.text(((p[0] + q[0]) / 2, p[1] + 16), label, fill=INK, font=F['dim'], anchor='ma')

    for b in bots:
        C_.paste(b['im'], (x, floor - b['h']))
        hx = x - 45
        D.line([(hx - 18, floor - b['h']), (x + 30, floor - b['h'])], fill=FAINT, width=2)
        D.line([(hx - 18, floor), (x + 30, floor)], fill=FAINT, width=2)
        arrow((hx, floor - b['h']), (hx, floor), f"{b['height'] * 100:.1f} cm", 'left')
        wy = floor + 28
        arrow((x, wy), (x + b['w'], wy), f"{b['width'] * 100:.1f} cm", 'below')
        cx = x + b['w'] / 2
        y = wy + 66
        D.text((cx, y), b['title'], fill=INK, font=F['title'], anchor='ma')
        y += 58
        D.text((cx, y), f"{b['mass']:.2f} kg", fill=INK, font=F['title'], anchor='ma')
        y += 58
        for s in b['servos']:
            D.text((cx, y), s, fill=LINE, font=F['text'], anchor='ma')
            y += 40
        print(b['title'], f"{b['height'] * 100:.1f} cm tall, {b['width'] * 100:.1f} cm wide, {b['mass']:.2f} kg")
        x += b['w'] + GAP
    C_.save(os.path.join(OUT, 'recipes.png'), optimize=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
