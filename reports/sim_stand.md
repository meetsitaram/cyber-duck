# Stand test in MuJoCo

1.51 kg. 5 s holding its stance, leaned back at the ankles until balanced, servos at their peak-torque limit. Not tuned against a real robot.

Largest tilt 15.1 deg; root height change +13.9 mm.

| joint | servo | peak N m | RMS N m | rated N m | RMS / rated |
|---|---|---|---|---|---|
| L_hip_yaw | STS3215_C046 | 0.017 | 0.016 | 0.27 | 6 % |
| R_hip_yaw | STS3215_C046 | 0.016 | 0.016 | 0.27 | 6 % |
| L_hip_roll | STS3215_C044 | 0.054 | 0.053 | 0.35 | 15 % |
| R_hip_roll | STS3215_C044 | 0.053 | 0.052 | 0.35 | 15 % |
| L_hip_pitch | STS3215_C044 | 0.021 | 0.021 | 0.35 | 6 % |
| R_hip_pitch | STS3215_C044 | 0.021 | 0.020 | 0.35 | 6 % |
| L_knee | STS3215_C044 | 0.174 | 0.174 | 0.35 | 50 % |
| R_knee | STS3215_C044 | 0.173 | 0.173 | 0.35 | 49 % |
| L_ankle_pitch | STS3215_C044 | 0.077 | 0.076 | 0.35 | 22 % |
| R_ankle_pitch | STS3215_C044 | 0.077 | 0.076 | 0.35 | 22 % |
| neck_pitch | XC330_M288 | 0.022 | 0.022 | 0.31 | 7 % |
| head_pitch | XL330_M288 | 0.023 | 0.023 | 0.17 | 14 % |
| head_yaw | XL330_M288 | 0.001 | 0.001 | 0.17 | 0 % |
| head_roll | XL330_M288 | 0.000 | 0.000 | 0.17 | 0 % |
| beak | XL330_M288 | 0.016 | 0.016 | 0.17 | 10 % |
