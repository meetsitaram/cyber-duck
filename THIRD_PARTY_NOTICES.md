# Third-party notices

milli is licensed under the Apache License 2.0 (`LICENSE`). It is forked from echo (github.com/meetsitaram/echo).

## References for milli (nothing copied)

- microduck, Pollen Robotics (pollen-robotics/microduck_rl): joints, proportions and shapes were measured from its model.
  Its meshes are CC BY-NC-SA 4.0 and are not in this repo. Its identified XL330 dynamics (Apache-2.0) are in
  `spec/actuators.json`. `trainer/milli_trainer.patch` is a change to its Apache-2.0 code, and includes lines of it as context.
- SO-ARM100 (TheRobotStudio/SO-ARM100, Apache-2.0): the STS3215's size and mounting pattern.
- Open Duck Mini and Open Duck Playground (apirrone, Apache-2.0): STS3215 dynamics, and the idea of the TPU sole and foot switches.

## Bought parts' CAD inside the Fusion models and their exports

`cad/duck-droids-*.f3d` and `.step` contain the makers' models of the servos, so that the model shows the real part. The STEP files themselves are not in this repo: `tools/fetch_servo_cad.py` fetches them.

- XL330-M288-T and FPX330-H101 idler: ROBOTIS's models, as published in Koch v1.1 (jess-moss/koch-v1-1, Apache-2.0, Copyright 2024 The Hugging Face team), which this project's owner has forked. That repository's licence is the one its files came under; ROBOTIS's own terms for its CAD were looked for and not found, so ROBOTIS may keep rights in them. Remove them from the exports if ROBOTIS asks.
- STS3215 and its passive horn: SO-ARM100 (TheRobotStudio/SO-ARM100, Apache-2.0).
- HD-1910-C001, with its horns: Feetech's own model, HD-1910-C001-20260902.stp, published on the FEETECH wiki (ftservo.github.io/ftservo-wiki). No licence terms were found with it, so Feetech may keep rights in it. Remove it from the exports if Feetech asks.
- RS04, RS03, RS00, power board and CAN hub: RobStride's models, from github.com/RobStride/Product_Information, read and measured for the big duck. That repository has no licence file, so they're not in this repo, and no `cad/duck-droids-big` export exists. The numbers measured from them are in `spec/big_duck.py`.

## Adapted into echo: GMR, General Motion Retargeting (MIT licence)

- Source: https://github.com/YanjieZe/GMR
- Used in:
  - `tools/ik/pico_to_echo.json`: the IK table, derived from GMR's `xrobot_to_g1.json` (by way of the X2 table in
    github.com/meetsitaram/agibot-x2-record-and-replay).
  - `tools/retarget_pico.py`: the two-stage IK solve loop, after GMR's `motion_retarget.py`.
- Changes: echo's body names and reference bodies, toe bodies, echo's elbow range, offsets recalibrated per take,
  planted-foot locking and grounding.

> Copyright 2025 Yanjieze
>
> Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
> documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
> rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
> permit persons to whom the Software is furnished to do so, subject to the following conditions:
>
> The above copyright notice and this permission notice shall be included in all copies or substantial portions of
> the Software.
>
> THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO
> THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
> AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
> TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
> SOFTWARE.

The rest of the retargeter (capture-to-robot transform, segment morph, arm-plane frames, elbow posture task,
rest-frame calibration) is ported from the author's own scripts, in agibot-x2-record-and-replay and in the author's
fork of NVIDIA's GR00T-WholeBodyControl (github.com/meetsitaram/GR00T-WholeBodyControl). No NVIDIA-authored code is
included.


## Used as tools, not included

| tool | used for | licence |
|---|---|---|
| NLF, Neural Localizer Fields, github.com/isarandi/nlf | reading the robot's body pose in each video frame (`tools/video_nlf.py`) | code MIT; models for non-commercial research use; neither is in this repository |
| PyTorch, torchvision, OpenCV | running NLF | their own licences |

## Consulted, not copied

These were read for comparison or as design references. None of their files, code, meshes or models are in echo, so
their licences place no conditions on it.

| reference | what we took from it | licence |
|---|---|---|
| Asimov 1, github.com/menloresearch/asimov-1 | the internal layout: one actuator per joint, hip pitch-roll-yaw, yaw-only waist, parallel ankle; joint orders and ranges read from its URDF | hardware CERN-OHL-S-2.0, software GPL-2.0 |
| Unitree G1, H1-2, H2 URDFs, github.com/unitreerobotics/unitree_ros | joint orders, offsets and ranges, for comparison | BSD-3-Clause |
| AgiBot A3, github.com/AgibotTech/A3-A3U-robot-model; AgiBot X2, github.com/AgibotTech/agibot_x2_urdf | joint orders, offsets and ranges, for comparison | Mulan PSL v2 |

## Trademarks and the reference robot

Figure and Figure 03 are trademarks of Figure AI, Inc. echo is an independent project, neither affiliated with nor
endorsed by Figure AI. Figure 03's published specifications (figure.ai) and frames of its reveal video were used to
compare proportions; the frames are not in this repository, and echo uses its own mark, not Figure's logo. The
living-room motion was measured from Figure's public home-work video (x.com/Figure_robot, 2026-09-17); the video, its
frames and the poses read from it are not in this repository, only echo's retargeted motion and our own renders.

RobStride and its actuator model names, PICO and XRoboToolkit, Unitree, AgiBot, Asimov, Autodesk Fusion and MuJoCo
belong to their owners and are named only to say what was used.

## Libraries

echo's tools import numpy, SciPy, trimesh, manifold3d, MuJoCo, mink, Pillow, imageio and matplotlib, each under its own
licence. They are installed separately and not redistributed here.

- Product photos in `docs/dynamixel/parts/`: Waveshare Bus Servo Adapter (A), OVONIC 2S LiPo, hiBCTR USB buck converter, Raspberry Pi Zero, from their product listings. Each belongs to its owner.
