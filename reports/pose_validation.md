# Pose validation: microduck and milli at the same joint angles

microduck scaled x 1.20; the left foot planted. Each landmark's move from the zero pose, microduck's vs milli's (mm), and the gap between them. Joint signs from the world axes: left_hip_yaw -1, left_knee -1, right_hip_yaw -1, right_hip_pitch -1, right_ankle -1, neck_pitch -1, head_roll -1 (the rest +1).

| pose | landmark | microduck moves | milli moves | gap |
|---|---|---|---|---|
| INIT | head (roll joint) | 0 | 0 | 0 |
| INIT | body (hip yaws) | 0 | 0 | 0 |
| INIT | L knee | 0 | 0 | 0 |
| INIT | R knee | 0 | 0 | 0 |
| INIT | R ankle | 0 | 0 | 0 |
| STAND | head (roll joint) | 60 | 40 | 21 |
| STAND | body (hip yaws) | 39 | 44 | 5 |
| STAND | L knee | 23 | 24 | 1 |
| STAND | R knee | 27 | 28 | 2 |
| STAND | R ankle | 20 | 21 | 2 |
| STAND | (milli's limits) | | | clipped: neck_pitch -0.35 |
| SIT | head (roll joint) | 276 | 313 | 141 |
| SIT | body (hip yaws) | 69 | 83 | 14 |
| SIT | L knee | 0 | 0 | 0 |
| SIT | R knee | 0 | 0 | 0 |
| SIT | R ankle | 0 | 0 | 0 |
| SIT | (milli's limits) | | | clipped: neck_pitch -0.50 |
| FOLD | head (roll joint) | 118 | 205 | 179 |
| FOLD | body (hip yaws) | 71 | 68 | 29 |
| FOLD | L knee | 0 | 0 | 0 |
| FOLD | R knee | 0 | 0 | 0 |
| FOLD | R ankle | 0 | 0 | 0 |
| FOLD | (milli's limits) | | | clipped: L_hip_pitch +1.57, L_knee -1.57, neck_pitch -1.00, R_hip_pitch +1.57, R_knee -1.57 |
| hip pitch +0.5 (leg forward) | head (roll joint) | 79 | 74 | 25 |
| hip pitch +0.5 (leg forward) | body (hip yaws) | 8 | 9 | 1 |
| hip pitch +0.5 (leg forward) | L knee | 0 | 0 | 0 |
| hip pitch +0.5 (leg forward) | R knee | 25 | 26 | 1 |
| hip pitch +0.5 (leg forward) | R ankle | 44 | 42 | 3 |
| hip pitch +0.5 (leg forward) | (milli's limits) | | | clipped: L_hip_pitch +0.50 |
| knee +0.6 | head (roll joint) | 114 | 131 | 34 |
| knee +0.6 | body (hip yaws) | 36 | 43 | 7 |
| knee +0.6 | L knee | 0 | 0 | 0 |
| knee +0.6 | R knee | 0 | 0 | 0 |
| knee +0.6 | R ankle | 30 | 31 | 2 |
| ankle +0.5 | head (roll joint) | 120 | 134 | 29 |
| ankle +0.5 | body (hip yaws) | 51 | 57 | 7 |
| ankle +0.5 | L knee | 25 | 26 | 1 |
| ankle +0.5 | R knee | 25 | 26 | 1 |
| ankle +0.5 | R ankle | 0 | 0 | 0 |
| hip roll +0.3, hip yaw +0.3 | head (roll joint) | 50 | 66 | 17 |
| hip roll +0.3, hip yaw +0.3 | body (hip yaws) | 10 | 10 | 2 |
| hip roll +0.3, hip yaw +0.3 | L knee | 0 | 0 | 0 |
| hip roll +0.3, hip yaw +0.3 | R knee | 36 | 35 | 5 |
| hip roll +0.3, hip yaw +0.3 | R ankle | 54 | 53 | 7 |
| neck pitch +0.6, head pitch -0.6 | head (roll joint) | 80 | 59 | 56 |
| neck pitch +0.6, head pitch -0.6 | body (hip yaws) | 0 | 0 | 0 |
| neck pitch +0.6, head pitch -0.6 | L knee | 0 | 0 | 0 |
| neck pitch +0.6, head pitch -0.6 | R knee | 0 | 0 | 0 |
| neck pitch +0.6, head pitch -0.6 | R ankle | 0 | 0 | 0 |
| neck pitch +0.6, head pitch -0.6 | (milli's limits) | | | clipped: neck_pitch -0.60 |
| head yaw +1.0, head roll +0.3 | head (roll joint) | 21 | 18 | 38 |
| head yaw +1.0, head roll +0.3 | body (hip yaws) | 0 | 0 | 0 |
| head yaw +1.0, head roll +0.3 | L knee | 0 | 0 | 0 |
| head yaw +1.0, head roll +0.3 | R knee | 0 | 0 | 0 |
| head yaw +1.0, head roll +0.3 | R ankle | 0 | 0 | 0 |
