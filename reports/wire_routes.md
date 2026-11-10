# Wire routes: hybrid

From `tools/wire_routes.py`. Longest straight run through the points in any pose, at each end of each joint crossed; need adds 15 % and 30 mm. Not a build: measure on one.

| Wire | What | Longest run, mm | Need, mm | Stock, mm | Joints it crosses |
|---|---|---|---|---|---|
| left ankle to knee | STS3215 lead | 71 | 111 | 150 | L knee |
| left knee to hip pitch | STS3215 lead | 41 | 77 | 150 | none |
| left hip pitch to hip roll | STS3215 lead | 70 | 111 | 150 | L hip pitch, L hip roll |
| left hip roll to hip yaw | STS3215 lead | 76 | 118 | 150 | L hip yaw |
| left hip yaw to splice | STS3215 lead, one plug cut off | 19 | 52 | 150 | none |
| right ankle to knee | STS3215 lead | 71 | 111 | 150 | R knee |
| right knee to hip pitch | STS3215 lead | 41 | 77 | 150 | none |
| right hip pitch to hip roll | STS3215 lead | 70 | 111 | 150 | R hip pitch, R hip roll |
| right hip roll to hip yaw | STS3215 lead | 76 | 118 | 150 | R hip yaw |
| right hip yaw to splice | STS3215 lead, one plug cut off | 19 | 52 | 150 | none |
| battery to splice | XT30 pigtail, 16 AWG: V and G | 79 | 121 | cut to length | none |
| battery to UBEC | 20 AWG pair | 41 | 77 | cut to length | none |
| Feetech board to splice | D and G: 5264 plug and 28 AWG, up the neck | 258 | 327 | cut to length | head pitch, head roll, head yaw, neck pitch |
| 2S to the Feetech board | 20 AWG pair, up the neck | 298 | 373 | cut to length | head pitch, head roll, head yaw, neck pitch |
| 5 V to the Dynamixel board | 20 AWG pair, up the neck | 306 | 382 | cut to length | head pitch, head roll, head yaw, neck pitch |
| IMU | 4 x 28 AWG, down the neck | 408 | 499 | cut to length | head pitch, head roll, head yaw, neck pitch |
| UART | 3 x 28 AWG | 130 | 180 | cut to length | none |
| Pi's regulator input | 2 x 28 AWG | 100 | 145 | cut to length | none |
| Pi's power | LY tech USB ribbon | 36 | 71 | 100 | none |
| Pi to the Dynamixel board | Veeloon OTG lead | 76 | 118 | 183 | none |
| camera | ribbon | 48 | 85 | 150 | none |
| Dynamixel board to beak | ROBOTIS lead | 65 | 105 | 180 | none |
| beak to head roll | ROBOTIS lead | 33 | 68 | 180 | head roll |
| head roll to head yaw | ROBOTIS lead | 57 | 95 | 180 | head yaw |
| head yaw to head pitch | ROBOTIS lead | 85 | 127 | 180 | head pitch |
| head pitch to neck | ROBOTIS lead | 39 | 75 | 180 | none |

26 wires, 0 failing
