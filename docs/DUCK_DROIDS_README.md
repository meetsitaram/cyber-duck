# duck-droids

Open biped robot ducks you can build from servos you can buy, each generated from one Python spec. They take their look from Pollen Robotics' microduck, and aren't made by or connected with Pollen.

![The all-Dynamixel duck assembling itself: motors, skeleton, electronics, covers](https://github.com/meetsitaram/duck-droids/blob/main/docs/images/assembly/dynamixel/reveal.gif)

**Watch:** [30-second reveal](https://github.com/meetsitaram/duck-droids/blob/main/docs/images/assembly/dynamixel/reveal_30s.mp4) | [5-minute build](https://github.com/meetsitaram/duck-droids/blob/main/docs/images/assembly/dynamixel/build_5min.mp4)

> **Building one? Start with the all-Dynamixel duck.** Every duck has its own parts list, print files and wiring, in `docs/<duck>/` and `prints/<duck>/`, and they don't mix: order and print from one duck's folder only.

## The all-Dynamixel duck (start here)

29.5 cm tall, 1.00 kg, 15 ROBOTIS XL330 servos, a Raspberry Pi Zero 2 W in the head.

![The all-Dynamixel duck assembling itself: motors, skeleton, electronics, covers](https://github.com/meetsitaram/duck-droids/blob/main/docs/images/assembly/dynamixel/reveal.gif)

**Watch:** [30-second reveal](https://github.com/meetsitaram/duck-droids/blob/main/docs/images/assembly/dynamixel/reveal_30s.mp4) | [5-minute build](https://github.com/meetsitaram/duck-droids/blob/main/docs/images/assembly/dynamixel/build_5min.mp4)

| | |
|---|---|
| Parts to buy | [BOM](https://github.com/meetsitaram/duck-droids/blob/main/docs/dynamixel/BOM.md) |
| Files to print | [STLs](https://github.com/meetsitaram/duck-droids/blob/main/prints/dynamixel/README.md) |
| How to build it | [Assembly guide](https://github.com/meetsitaram/duck-droids/blob/main/docs/dynamixel/ASSEMBLY.md), [wiring and every part](https://github.com/meetsitaram/duck-droids/blob/main/docs/dynamixel/WIRING.md) |
| CAD | [Fusion file](https://github.com/meetsitaram/duck-droids/blob/main/cad/duck-droids-dynamixel.f3d), [STEP](https://github.com/meetsitaram/duck-droids/blob/main/cad/duck-droids-dynamixel.step) |
| Run it | [Pi code](https://github.com/meetsitaram/duck-droids/blob/main/pi/README.md) |
| Teach it to walk | [Training](https://github.com/meetsitaram/duck-droids/blob/main/docs/TRAINING.md) |

**Status:** designed and checked in software. Nothing has been printed or built yet. Before ordering everything, print one joint and bench-test the electronics: the list is in its [open questions](https://github.com/meetsitaram/duck-droids/blob/main/docs/OPEN_QUESTIONS.md). In simulation it walks on its IMU alone, without reading its servos' angles, nearly as well as with them; that walk isn't deployed yet.

## Other ducks

The same design on other servos, and at 1.8 m. Each has its own parts list, prints and wiring.

![The four small ducks front on at one scale, with height, width, weight and servos: all Dynamixel 29.5 cm tall, 14.7 cm wide, 0.98 kg; Feetech HD-1910 29.5 cm, 14.7 cm, 1.03 kg; hybrid 31.8 cm, 18.0 cm, 1.65 kg; all Feetech 37.9 cm, 18.0 cm, 1.94 kg](https://github.com/meetsitaram/duck-droids/blob/main/docs/images/recipes/recipes.png)

![A 1.8 m white duck walking behind four small coloured ducks, each labelled with its servos, their beaks opening in turn](https://github.com/meetsitaram/duck-droids/blob/main/docs/images/family/duck_family.gif)

The family ([MP4, 17 s, sharper](https://github.com/meetsitaram/duck-droids/blob/main/docs/images/family/duck_family.mp4)), in simulation only, from walks trained on 3 November, before the foot switches were taken out: today's designs have no trained walk yet. The hobby-servo and nano ducks aren't in it yet.

![All seven ducks standing to scale in a row in a room, from the 21 cm nano duck to the 1.8 m big duck beside a sofa, each labelled](https://github.com/meetsitaram/duck-droids/blob/main/docs/images/family_lineup.png)

All seven to scale ([MP4, 62 s](https://github.com/meetsitaram/duck-droids/blob/main/docs/images/family_lineup.mp4)): the camera goes along the row, and each duck nods, bites and turns, with its servos, height, width and weight. Made by `tools/lineup_video.py`; the motion is scripted, not a trained walk.

### Feetech HD-1910 duck

29.5 cm, 1.03 kg, 15 Feetech HD-1910 servos: the all-Dynamixel duck's layout on the servo the community's Microduck builds use.

[BOM](https://github.com/meetsitaram/duck-droids/blob/main/docs/hd1910/BOM.md) · [STLs](https://github.com/meetsitaram/duck-droids/blob/main/prints/hd1910/README.md) · [assembly](https://github.com/meetsitaram/duck-droids/blob/main/docs/hd1910/ASSEMBLY.md) · [wiring](https://github.com/meetsitaram/duck-droids/blob/main/docs/hd1910/WIRING.md) · [Fusion](https://github.com/meetsitaram/duck-droids/blob/main/cad/duck-droids-hd1910.f3d) · [30-second reveal](https://github.com/meetsitaram/duck-droids/blob/main/docs/images/assembly/hd1910/reveal_30s.mp4)

### All-Feetech duck

37.9 cm, 1.94 kg, 15 Feetech STS3215 servos, the SO-101 arm's.

[BOM](https://github.com/meetsitaram/duck-droids/blob/main/docs/feetech/BOM.md) · [STLs](https://github.com/meetsitaram/duck-droids/blob/main/prints/feetech/README.md) · [assembly](https://github.com/meetsitaram/duck-droids/blob/main/docs/feetech/ASSEMBLY.md) · [wiring](https://github.com/meetsitaram/duck-droids/blob/main/docs/feetech/WIRING.md) · [Fusion](https://github.com/meetsitaram/duck-droids/blob/main/cad/duck-droids-feetech.f3d) · [30-second reveal](https://github.com/meetsitaram/duck-droids/blob/main/docs/images/assembly/feetech/reveal_30s.mp4)

### Hybrid duck

31.8 cm, 1.65 kg: STS3215 legs, Dynamixel XC330 and XL330 neck and head, on two buses.

[BOM](https://github.com/meetsitaram/duck-droids/blob/main/docs/hybrid/BOM.md) · [STLs](https://github.com/meetsitaram/duck-droids/blob/main/prints/hybrid/README.md) · [assembly](https://github.com/meetsitaram/duck-droids/blob/main/docs/hybrid/ASSEMBLY.md) · [wiring](https://github.com/meetsitaram/duck-droids/blob/main/docs/hybrid/WIRING.md) · [Fusion](https://github.com/meetsitaram/duck-droids/blob/main/cad/duck-droids-hybrid.f3d) · [30-second reveal](https://github.com/meetsitaram/duck-droids/blob/main/docs/images/assembly/hybrid/reveal_30s.mp4)

### Hobby-servo duck

37.3 cm, 1.94 kg, 15 one-way PWM servos (9 MG996R, 3 DS3218, 3 MG90S) on a PCA9685 board, all the electronics in the body. Hobby servos don't report their angles, so it would walk on its IMU alone. Spec, checks and docs only; no Fusion model yet.

[BOM](https://github.com/meetsitaram/duck-droids/blob/main/docs/hobby/BOM.md) · [STLs](https://github.com/meetsitaram/duck-droids/blob/main/prints/hobby/README.md) · [assembly](https://github.com/meetsitaram/duck-droids/blob/main/docs/hobby/ASSEMBLY.md) · [wiring](https://github.com/meetsitaram/duck-droids/blob/main/docs/hobby/WIRING.md)

### Nano duck

20.6 cm, 370 g, 13 SG90 and MG90S micro servos driven by the Pi's own PWM. The smallest these servos allow. Spec, checks and docs only; its looks still need work.

[BOM](https://github.com/meetsitaram/duck-droids/blob/main/docs/nano/BOM.md) · [STLs](https://github.com/meetsitaram/duck-droids/blob/main/prints/nano/README.md) · [assembly](https://github.com/meetsitaram/duck-droids/blob/main/docs/nano/ASSEMBLY.md) · [wiring](https://github.com/meetsitaram/duck-droids/blob/main/docs/nano/WIRING.md)

### Big duck: **not safe to build**

1.8 m, 98.6 kg, 22 RobStride actuators. It has its Fusion links, a [30-second reveal](https://github.com/meetsitaram/duck-droids/blob/main/docs/images/assembly/big/reveal_30s.mp4) and a [5-minute build](https://github.com/meetsitaram/duck-droids/blob/main/docs/images/assembly/big/build_5min.mp4). In simulation it stays up and walks slowly, about 0.2 m/s.

[BOM](https://github.com/meetsitaram/duck-droids/blob/main/docs/big/BOM.md) · [wiring](https://github.com/meetsitaram/duck-droids/blob/main/docs/big/WIRING.md) · [rebuild, render and train it](https://github.com/meetsitaram/duck-droids/blob/main/docs/big/REPRODUCE.md)

## Where things are

| Folder | What |
|---|---|
| `docs/<duck>/` | that duck's BOM, assembly guide and wiring |
| `prints/<duck>/` | that duck's STLs, already turned to print, and their README |
| `cad/` | each duck's Fusion file and STEP |
| `spec/` | the Python spec every duck is generated from |
| `tools/` | checks, exports, docs and renders |
| `pi/` | the all-Dynamixel duck's Pi code and its simulator |

More in [the details](https://github.com/meetsitaram/duck-droids/blob/main/docs/DETAILS.md), [the open questions](https://github.com/meetsitaram/duck-droids/blob/main/docs/OPEN_QUESTIONS.md), and [what went wrong along the way](https://github.com/meetsitaram/duck-droids/blob/main/LESSONS.md).

## Run the checks

```bash
MILLI_RECIPE=dynamixel python -I spec/robot.py
```

```bash
MILLI_RECIPE=dynamixel python -I tools/checks.py
```

```bash
MILLI_RECIPE=dynamixel python -I tools/assembly.py
```

Python 3.12 with numpy, scipy, trimesh, manifold3d, shapely, Pillow, imageio and mujoco.

## Credits

Forked from [echo](https://github.com/meetsitaram/echo). Look and joint layout measured from [microduck's model](https://github.com/pollen-robotics/microduck_rl), whose meshes aren't in this repo. Servo mounting after the [SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100), servo dynamics after [Open Duck Mini](https://github.com/apirrone/Open_Duck_Mini). See [THIRD_PARTY_NOTICES.md](https://github.com/meetsitaram/duck-droids/blob/main/THIRD_PARTY_NOTICES.md).

Apache-2.0: see [LICENSE](https://github.com/meetsitaram/duck-droids/blob/main/LICENSE).
