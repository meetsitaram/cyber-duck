"""Cyber Duck's movie poster (the user, 2026-11-10: "like a movie poster - transformers - the age of cyberduck"):
the duck towering in robot mode, the truck parked in front, rim-lit in orange and teal on a dark cinematic ground,
chrome title lettering, a tagline and a credits block. Portrait 1080 x 1920, to open the desk clip.

  MILLI_RECIPE=dynamixel python tools/cyber_duck_poster.py   -> reports/<recipe>/cyber_duck/poster.png

The robot is rendered from the spec's shapes and the Cyber shells (MuJoCo), cut out by its depth and composited;
the lettering is original (not the franchise's logo). Plain python.
"""
import math
import os
import pickle
import sys
import tempfile

import mujoco
import numpy as np
from PIL import Image, ImageChops, ImageDraw, ImageFilter, ImageFont

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import memcap  # noqa: E402
import cyber_duck as CD  # noqa: E402
import cyber_duck_render as DR  # noqa: E402
import geom as G  # noqa: E402
import render_pose as RP  # noqa: E402

W, H = 1080, 1920
FONTS = 'C:/Windows/Fonts/'


class PosterScene(DR.DeskScene):
    """The robot alone in the dark, lit from behind in orange and teal, a cool key from the front."""

    def xml(self, q, tmp, offset=(0.0, 0.0), exact=None, cover=1.0):
        x = super().xml(q, tmp, offset, exact, cover)
        a, b = x.index('<worldbody>'), x.index('</worldbody>')
        robot = x[x.index('<geom type="mesh"', a):b]
        lights = """
<light pos="-0.5 0.6 0.35" dir="0.6 -0.6 -0.2" diffuse="1.0 0.50 0.16" specular="1.0 0.55 0.25" castshadow="false"/>
<light pos="0.6 0.6 0.4" dir="-0.6 -0.6 -0.25" diffuse="0.06 0.26 0.32" specular="0.15 0.45 0.55" castshadow="false"/>
<light pos="0.4 -0.9 0.25" dir="-0.4 0.9 0.05" diffuse="0.70 0.66 0.62" specular="0.9 0.85 0.8" castshadow="false"/>
<light pos="0 -0.3 1.2" dir="0 0.2 -1" diffuse="0.25 0.25 0.28" specular="0.3 0.3 0.3" castshadow="false"/>
"""
        x = x[:a] + '<worldbody>' + lights + robot + x[b:]
        x = x.replace('<texture name="sky" type="skybox"', '<texture name="skyunused" type="skybox"')
        return x.replace('<headlight ambient="0.22 0.22 0.23"', '<headlight ambient="0.10 0.10 0.12"')

    def cutout(self, q, theta, el, dist, target, w, h):
        """An RGBA render: the robot, its background transparent (by depth)."""
        global_w, global_h = DR.W, DR.H
        DR.W, DR.H = w, h
        try:
            with tempfile.TemporaryDirectory() as tmp:
                path = os.path.join(tmp, 'scene.xml')
                with open(path, 'w') as fh:
                    fh.write(self.xml(q, tmp))
                m = mujoco.MjModel.from_xml_path(path)
                d = mujoco.MjData(m)
                mujoco.mj_forward(m, d)
                cam = mujoco.MjvCamera()
                cam.type = mujoco.mjtCamera.mjCAMERA_FREE
                cam.lookat[:] = target
                cam.distance, cam.azimuth, cam.elevation = dist, 180.0 - theta, el
                r = mujoco.Renderer(m, h, w)
                r.update_scene(d, cam)
                rgb = r.render().copy()
                r.enable_depth_rendering()
                r.update_scene(d, cam)
                depth = r.render().copy()
                r.close()
        finally:
            DR.W, DR.H = global_w, global_h
        alpha = (depth < depth.max() - 1e-4).astype(np.uint8) * 255
        im = Image.fromarray(np.dstack([rgb, alpha]), 'RGBA')
        return im


def background():
    """A burnt-orange sunset fading to steel teal above, smoke rolling behind, embers, a lens-flare streak (the mood
    of the franchise's posters the user pointed to; nothing copied from them)."""
    rng = np.random.default_rng(7)
    y, x = np.mgrid[0:H, 0:W].astype(np.float32)
    v = y / H
    teal, orange, ember = np.array([18, 44, 58]), np.array([215, 95, 25]), np.array([255, 170, 70])
    t = np.clip((v - 0.05) / 0.6, 0, 1)[..., None]
    img = teal * (1 - t) + orange * t
    sun = np.exp(-(((x - W * 0.5) / 260) ** 2 + ((y - 1080) / 180) ** 2))
    img += sun[..., None] * (ember - img) * 0.9
    # smoke: blurred noise at a few scales, dark and warm
    smoke = np.zeros((H, W), np.float32)
    for scale, amp in ((64, 1.0), (24, 0.5), (8, 0.25)):
        n = rng.random((H // scale + 2, W // scale + 2)).astype(np.float32)
        n = np.asarray(Image.fromarray((n * 255).astype(np.uint8)).resize((W, H), Image.BICUBIC), np.float32) / 255
        smoke += amp * n
    smoke = (smoke / 1.75 - 0.35).clip(0, 1) * np.clip(1.4 - abs(v - 0.55) * 2.2, 0, 1)
    img = img * (1 - 0.65 * smoke[..., None]) + np.array([40, 26, 20]) * 0.65 * smoke[..., None]
    # the ground: dark, warm-lit at the horizon
    ground = np.clip((y - 1300) / 300, 0, 1)[..., None]
    img = img * (1 - ground) + np.array([14, 10, 9]) * ground
    # vignette
    vig = 1 - 0.55 * (((x - W / 2) / (W * 0.75)) ** 2 + ((y - H * 0.5) / (H * 0.7)) ** 2)
    img *= vig.clip(0.25, 1)[..., None]
    im = Image.fromarray(img.clip(0, 255).astype(np.uint8))
    d = ImageDraw.Draw(im)
    for _ in range(220):                                   # embers and debris
        px, py = rng.uniform(0, W), rng.uniform(150, 1450)
        r_ = rng.uniform(0.6, 2.8)
        hot = rng.random() > 0.3
        c = (255, int(rng.uniform(120, 210)), int(rng.uniform(30, 90))) if hot else (30, 26, 24)
        d.ellipse([px - r_, py - r_ * 0.8, px + r_, py + r_ * 0.8], fill=c)
    im = im.filter(ImageFilter.GaussianBlur(0.7))
    # a horizontal lens flare through the sun
    fl = np.zeros((H, W), np.float32)
    fl += np.exp(-((y - 1080) / 5) ** 2) * np.exp(-((x - W / 2) / 520) ** 2)
    fl += 0.5 * np.exp(-((y - 1080) / 22) ** 2) * np.exp(-((x - W / 2) / 260) ** 2)
    arr = np.asarray(im, np.float32) + fl[..., None] * np.array([255, 200, 140])
    return Image.fromarray(arr.clip(0, 255).astype(np.uint8))


def chrome_text(text, font, fill_top=(250, 250, 252), fill_bot=(120, 125, 135), stroke=3, glow=(255, 140, 50)):
    """Brushed-chrome lettering: a bright-to-dark gradient, a dark edge, a horizon line, a warm glow."""
    bbox = font.getbbox(text, stroke_width=stroke)
    w, h = bbox[2] - bbox[0] + 40, bbox[3] - bbox[1] + 40
    mask = Image.new('L', (w, h), 0)
    ImageDraw.Draw(mask).text((20 - bbox[0], 20 - bbox[1]), text, font=font, fill=255)
    edge = Image.new('L', (w, h), 0)
    ImageDraw.Draw(edge).text((20 - bbox[0], 20 - bbox[1]), text, font=font, fill=255, stroke_width=stroke, stroke_fill=255)
    g = np.linspace(0, 1, h)[:, None]
    horizon = 0.5 + 0.08 * np.sin(np.linspace(0, 3 * math.pi, w))[None, :]
    t = np.where(g < horizon, g / horizon * 0.45, 0.55 + (g - horizon) / (1 - horizon) * 0.45)
    top, bot = np.array(fill_top), np.array(fill_bot)
    grad = (top[None, None] * (1 - t[..., None]) + bot[None, None] * t[..., None])
    brushed = np.random.default_rng(1).normal(0, 6, (h, w))
    brushed = (brushed + np.roll(brushed, 1, 1) + np.roll(brushed, 2, 1) + np.roll(brushed, 3, 1)) / 4
    grad = (grad + brushed[..., None]).clip(0, 255).astype(np.uint8)
    out = Image.new('RGBA', (w, h), (0, 0, 0, 0))
    halo = Image.new('RGBA', (w, h), glow + (0,))
    halo.putalpha(edge.filter(ImageFilter.GaussianBlur(12)).point(lambda v: int(v * 0.55)))
    out = Image.alpha_composite(out, halo)
    dark = Image.new('RGBA', (w, h), (20, 22, 28, 255))
    dark.putalpha(edge)
    out = Image.alpha_composite(out, dark)
    face = Image.fromarray(grad).convert('RGBA')
    face.putalpha(mask)
    return Image.alpha_composite(out, face)


def paste_center(canvas, im, cy):
    canvas.alpha_composite(im, (int((W - im.width) / 2), int(cy - im.height / 2)))


def main():
    memcap.guard(cap_gb=6, need_free_gb=16)
    with open(os.path.join(CD.OUT, 'painted.pkl'), 'rb') as fh:
        data = pickle.load(fh)
    md = CD.Model()
    CD.attach_flaps(md, data['hinges'], data.get('jaw'))
    sc = PosterScene(md, data['extra'])
    sc.R = md.R
    canvas = background().convert('RGBA')

    # the duck, from below, bill open, head turned a little to us
    sc.base = np.eye(4)
    duck = sc.cutout({'jaw': 0.3, 'head_yaw': 0.25, 'neck_pitch': -0.1, 'head_pitch': -0.05}, 28, 12, 0.82,
                     (0.0, 0.0, 0.16), 1080, 1250)
    # standing on the ground: a soft contact shadow under its feet, then the duck
    sh = Image.new('L', (W, 140), 0)
    ImageDraw.Draw(sh).ellipse([300, 40, 780, 100], fill=210)
    sh = sh.filter(ImageFilter.GaussianBlur(18))
    shadow = Image.new('RGBA', (W, 140), (0, 0, 0, 255))
    shadow.putalpha(sh)
    canvas.alpha_composite(shadow, (0, 1222))
    paste_center(canvas, duck, 860)
    # the truck, in front, low three-quarter
    sc.base = CD.TRUCK_BASE
    Mt = md.R.fk(CD.TRUCK, base=CD.TRUCK_BASE)
    xs = np.concatenate([G.apply(Mt[ln], v)[:, 0] for ln, v, f, c, r in sc.parts if len(f)])
    truck = sc.cutout(CD.TRUCK, 32, 6, 0.72, (0.5 * (xs.min() + xs.max()), 0.0, 0.05), 1080, 560)
    # its reflection on a dark floor
    refl = truck.transpose(Image.FLIP_TOP_BOTTOM)
    a = np.asarray(refl).copy()
    fade = np.linspace(0.35, 0.0, a.shape[0])[:, None]
    a[..., 3] = (a[..., 3] * fade).astype(np.uint8)
    canvas.alpha_composite(Image.fromarray(a, 'RGBA'), (0, 1655))
    floor = Image.new('RGBA', (W, 520), (0, 0, 0, 0))
    fa = np.zeros((520, W, 4), np.uint8)
    fa[..., 3] = (np.linspace(0, 210, 520)[:, None]).astype(np.uint8)
    canvas.alpha_composite(Image.fromarray(fa, 'RGBA'), (0, 1420))
    paste_center(canvas, truck, 1385)

    # lettering
    d = ImageDraw.Draw(canvas)
    tag = ImageFont.truetype(FONTS + 'bahnschrift.ttf', 40)
    t1 = 'MORE THAN MEETS THE BILL'
    tw = d.textlength(t1, font=tag)
    d.text(((W - tw) / 2, 70), t1, font=tag, fill=(225, 225, 230))
    top = chrome_text('TRANSFORMERS', ImageFont.truetype(FONTS + 'impact.ttf', 150), stroke=4)
    paste_center(canvas, top, 1560)
    sub = chrome_text('THE AGE OF CYBER DUCK', ImageFont.truetype(FONTS + 'bahnschrift.ttf', 70), stroke=2,
                      fill_top=(255, 214, 170), fill_bot=(170, 90, 35), glow=(255, 120, 40))
    paste_center(canvas, sub, 1675)
    # (no credits block or COMING SOON: the user, 2026-11-10)
    out = os.path.join(CD.OUT, 'poster.png')
    canvas.convert('RGB').save(out)
    print(out)


if __name__ == '__main__':
    main()
