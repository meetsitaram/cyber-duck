"""The assembly guide's clips, cut section by section from the approved build video, so the guide shows the same thing it does.

  python tools/cut_clips.py        docs/images/assembly/<variant>/clips/<section>.mp4 and .gif (GitHub's page plays GIFs inline)

Section times come from reports/<variant>/build_sections.json, which tools/render_pov_video.py writes. If it's missing, they're worked out with dry runs at the same pace, and must land on the video's frame count.
Plain python (MuJoCo lives in the user site-packages).
"""
import json
import os
import subprocess
import sys

import imageio_ffmpeg

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import geom as G  # noqa: E402

MIRROR = ('thigh_R', 'shin_R', 'shoe_R', 'leg_R')      # the guide shows the left side only
GIF_W, GIF_FPS, GIF_SPEED = 300, 10, 1.5


def sections():
    p = G.out_path('reports', 'build_sections.json')
    if not os.path.exists(p):
        import render_pov_video as R
        pace = R.fit(300, False)
        n = R.run(False, 'dry', pace, False, True)
        with open(p, 'w', encoding='utf-8') as f:
            json.dump(dict(fps=R.FPS, frames=n, sections=R.LAST_MARKS), f, indent=0)
    return json.load(open(p, encoding='utf-8'))


def main():
    S = sections()
    ff = imageio_ffmpeg.get_ffmpeg_exe()
    d = os.path.join(G.ROOT, 'docs', 'images', 'assembly', G.RECIPE)
    src = os.path.join(d, 'build_5min.mp4')
    out = os.path.join(d, 'clips')
    os.makedirs(out, exist_ok=True)
    total = 0
    for key, (a, b) in S['sections'].items():
        if key in MIRROR:
            continue
        t0, dur = a / S['fps'], (b - a) / S['fps']
        mp4, gif = os.path.join(out, f'{key}.mp4'), os.path.join(out, f'{key}.gif')
        subprocess.run([ff, '-y', '-loglevel', 'error', '-ss', f'{t0:.3f}', '-i', src, '-t', f'{dur:.3f}', '-an', '-c:v', 'libx264', '-preset', 'slow', '-crf', '26',
                        '-pix_fmt', 'yuv420p', '-movflags', '+faststart', mp4], check=True)
        vf = f"setpts=PTS/{GIF_SPEED},fps={GIF_FPS},scale={GIF_W}:-2:flags=lanczos,split[a][b];[a]palettegen=max_colors=64:stats_mode=diff[p];[b][p]paletteuse=dither=none"
        # (-t before -i, so it limits the source: after -i, the speed-up would run on into the next section)
        subprocess.run([ff, '-y', '-loglevel', 'error', '-ss', f'{t0:.3f}', '-t', f'{dur:.3f}', '-i', src, '-an', '-filter_complex', vf, gif], check=True)
        total += os.path.getsize(mp4) + os.path.getsize(gif)
        print(f'{key:12s} {dur:5.1f} s  mp4 {os.path.getsize(mp4) // 1024:5d} kB  gif {os.path.getsize(gif) // 1024:5d} kB')
    print(f'{total // 1024} kB ->', out)


if __name__ == '__main__':
    main()
