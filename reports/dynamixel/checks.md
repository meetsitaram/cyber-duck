# Physical checks

spec: `spec/robot.json`, 2026-11-10 08:54, 45 s

181 checks: 178 pass, 3 warn, **0 fail**

## targets

| status | item | detail |
|---|---|---|
| WARN | height | 0.305 m: 1.15 x microduck's 0.264 m (proportions are checked at this scale) |
| PASS | head_length_m | 0.150 (microduck 0.123 x 1.15 = 0.142 +/- 10 %) |
| PASS | head_width_m | 0.105 (microduck 0.092 x 1.15 = 0.106 +/- 10 %) |
| PASS | head_height_m | 0.069 (microduck 0.059 x 1.15 = 0.068 +/- 10 %) |
| PASS | head_bottom_z | 0.235 (microduck 0.205 x 1.15 = 0.237 +/- 10 %) |
| PASS | neck_pitch_z | 0.157 (microduck 0.139 x 1.15 = 0.160 +/- 10 %) |
| PASS | hip_pitch_z | 0.100 (microduck 0.089 x 1.15 = 0.103 +/- 10 %) |
| PASS | body_width_m | 0.074 (microduck 0.064 x 1.15 = 0.074 +/- 10 %) |
| PASS | overall_width_m | 0.147 (microduck 0.141 x 1.15 = 0.163 +/- 10 %) |
| PASS | foot_length_m | 0.058 (microduck 0.054 x 1.15 = 0.062 +/- 10 %) |

## mass

| status | item | detail |
|---|---|---|
| PASS | total | 0.962 kg bottom-up (budget 1.55 kg; microduck 0.8 kg) |

## printable

| status | item | detail |
|---|---|---|
| PASS | largest part | 243 printed parts; largest head: 150 mm, on a 256 mm bed |

## axis

| status | item | detail |
|---|---|---|
| PASS | L_hip_yaw | XL330_M288 0.0 mm off the axis |
| PASS | R_hip_yaw | XL330_M288 0.0 mm off the axis |
| PASS | L_hip_roll | XL330_M288 0.0 mm off the axis |
| PASS | R_hip_roll | XL330_M288 0.0 mm off the axis |
| PASS | L_hip_pitch | XL330_M288 0.0 mm off the axis |
| PASS | R_hip_pitch | XL330_M288 0.0 mm off the axis |
| PASS | L_knee | XL330_M288 0.0 mm off the axis |
| PASS | R_knee | XL330_M288 0.0 mm off the axis |
| PASS | L_ankle_pitch | XL330_M288 0.0 mm off the axis |
| PASS | R_ankle_pitch | XL330_M288 0.0 mm off the axis |
| PASS | neck_pitch | XL330_M288 0.0 mm off the axis |
| PASS | head_pitch | XL330_M288 0.0 mm off the axis |
| PASS | head_yaw | XL330_M288 0.0 mm off the axis |
| PASS | head_roll | XL330_M288 0.0 mm off the axis |
| PASS | beak | XL330_M288 0.0 mm off the axis |

## bearing

| status | item | detail |
|---|---|---|
| PASS | L hip yaw horn bearing outer race | 0.0 mm off the axis, on trunk |
| PASS | R hip yaw horn bearing outer race | 0.0 mm off the axis, on trunk |
| PASS | L hip yaw horn bearing inner race | 0.0 mm off the axis, on L_hip_yaw_link |
| PASS | R hip yaw horn bearing inner race | 0.0 mm off the axis, on R_hip_yaw_link |
| PASS | L hip pitch horn bearing inner race | 0.0 mm off the axis, on L_hip_roll_link |
| PASS | R hip pitch horn bearing inner race | 0.0 mm off the axis, on R_hip_roll_link |
| PASS | L hip pitch horn bearing outer race | 0.0 mm off the axis, on L_hip_pitch_link |
| PASS | L knee horn bearing outer race | 0.0 mm off the axis, on L_hip_pitch_link |
| PASS | R hip pitch horn bearing outer race | 0.0 mm off the axis, on R_hip_pitch_link |
| PASS | R knee horn bearing outer race | 0.0 mm off the axis, on R_hip_pitch_link |
| PASS | L knee horn bearing inner race | 0.0 mm off the axis, on L_knee_link |
| PASS | R knee horn bearing inner race | 0.0 mm off the axis, on R_knee_link |
| PASS | head yaw horn bearing outer race | 0.0 mm off the axis, on head_pitch_link |
| PASS | head yaw horn bearing inner race | 0.0 mm off the axis, on head_yaw_link |
| PASS | head roll horn bearing outer race | 0.0 mm off the axis, on head_yaw_link |
| PASS | head roll rear bearing outer race | 0.0 mm off the axis, on head_yaw_link |
| PASS | head roll horn bearing inner race | 0.0 mm off the axis, on head |
| PASS | head roll rear bearing inner race | 0.0 mm off the axis, on head |
| PASS | L hip yaw horn races | on ['trunk', 'L_hip_yaw_link'] |
| PASS | R hip yaw horn races | on ['trunk', 'R_hip_yaw_link'] |
| PASS | L hip pitch horn races | on ['L_hip_roll_link', 'L_hip_pitch_link'] |
| PASS | R hip pitch horn races | on ['R_hip_roll_link', 'R_hip_pitch_link'] |
| PASS | L knee horn races | on ['L_hip_pitch_link', 'L_knee_link'] |
| PASS | R knee horn races | on ['R_hip_pitch_link', 'R_knee_link'] |
| PASS | head yaw horn races | on ['head_pitch_link', 'head_yaw_link'] |
| PASS | head roll horn races | on ['head_yaw_link', 'head'] |
| PASS | head roll rear races | on ['head_yaw_link', 'head'] |

## static

| status | item | detail |
|---|---|---|
| PASS | zero pose | 0 overlaps |

## covers

| status | item | detail |
|---|---|---|
| PASS | motors clear of the covers | 0 cover / motor crossings |

## connect

| status | item | detail |
|---|---|---|
| PASS | trunk structure |  |
| PASS | trunk covers mounted |  |
| PASS | L_hip_yaw_link structure |  |
| PASS | L_hip_yaw_link covers mounted |  |
| PASS | R_hip_yaw_link structure |  |
| PASS | R_hip_yaw_link covers mounted |  |
| PASS | L_hip_roll_link structure |  |
| PASS | L_hip_roll_link covers mounted |  |
| PASS | R_hip_roll_link structure |  |
| PASS | R_hip_roll_link covers mounted |  |
| PASS | L_hip_pitch_link structure |  |
| PASS | L_hip_pitch_link covers mounted |  |
| PASS | R_hip_pitch_link structure |  |
| PASS | R_hip_pitch_link covers mounted |  |
| PASS | L_knee_link structure |  |
| PASS | L_knee_link covers mounted |  |
| PASS | R_knee_link structure |  |
| PASS | R_knee_link covers mounted |  |
| PASS | L_foot structure |  |
| PASS | L_foot covers mounted |  |
| PASS | R_foot structure |  |
| PASS | R_foot covers mounted |  |
| PASS | neck_link structure |  |
| PASS | neck_link covers mounted |  |
| PASS | head_pitch_link structure |  |
| PASS | head_pitch_link covers mounted |  |
| PASS | head_yaw_link structure |  |
| PASS | head_yaw_link covers mounted |  |
| PASS | head structure |  |
| PASS | head covers mounted |  |
| PASS | beak_link structure |  |
| PASS | beak_link covers mounted |  |
| PASS | L_hip_yaw axis |  |
| PASS | R_hip_yaw axis |  |
| PASS | L_hip_roll axis |  |
| PASS | R_hip_roll axis |  |
| PASS | L_hip_pitch axis |  |
| PASS | R_hip_pitch axis |  |
| PASS | L_knee axis |  |
| PASS | R_knee axis |  |
| PASS | L_ankle_pitch axis |  |
| PASS | R_ankle_pitch axis |  |
| PASS | neck_pitch axis |  |
| PASS | head_pitch axis |  |
| PASS | head_yaw axis |  |
| PASS | head_roll axis |  |
| PASS | beak axis |  |

## skin

| status | item | detail |
|---|---|---|
| PASS | top plate | 0.00 % outside the skin |
| PASS | battery 2S 450 mAh | 0.00 % outside the skin |
| PASS | IMU | 0.00 % outside the skin |
| PASS | battery strap | 0.00 % outside the skin |
| PASS | body post 0 | 0.00 % outside the skin |
| PASS | body post 1 | 0.00 % outside the skin |
| PASS | power switch | 0.00 % outside the skin |
| PASS | power switch leads | 0.00 % outside the skin |
| PASS | head frame | 0.00 % outside the skin |
| PASS | head top plate wide | 0.00 % outside the skin |
| PASS | L head rail | 0.00 % outside the skin |
| PASS | R head rail | 0.00 % outside the skin |
| PASS | R head hinge boss | 0.00 % outside the skin |
| PASS | Pi mount | 0.00 % outside the skin |
| PASS | Pi mount 2 | 0.00 % outside the skin |
| PASS | Pi plate | 0.00 % outside the skin |
| PASS | Pi boss 0 | 0.00 % outside the skin |
| PASS | Pi boss 1 | 0.00 % outside the skin |
| PASS | Pi boss 2 | 0.00 % outside the skin |
| PASS | Pi boss 3 | 0.00 % outside the skin |
| PASS | Pi Zero 2 W | 0.00 % outside the skin |
| PASS | camera | 0.00 % outside the skin |
| PASS | Pi board tab | 0.00 % outside the skin |
| PASS | bus board standoff 0 | 0.00 % outside the skin |
| PASS | bus board standoff 1 | 0.00 % outside the skin |
| PASS | bus board standoff 2 | 0.00 % outside the skin |
| PASS | bus board standoff 3 | 0.00 % outside the skin |
| PASS | Waveshare bus board | 0.00 % outside the skin |
| PASS | L base riser top | 0.00 % outside the skin |
| PASS | head 5 V regulator | 0.00 % outside the skin |
| PASS | head regulator wires | 0.00 % outside the skin |
| PASS | Pi base tab | 0.00 % outside the skin |

## sweep

| status | item | detail |
|---|---|---|
| PASS | L_hip_yaw | [-0.45, +0.45] rad clear |
| PASS | L_hip_roll | [-0.33, +0.33] rad clear |
| PASS | L_hip_pitch | [-1.57, +0.95] rad clear |
| PASS | L_knee | [-1.10, +1.57] rad clear |
| PASS | L_ankle_pitch | [-1.50, +1.53] rad clear |
| PASS | neck_pitch | [-1.05, +1.60] rad clear |
| WARN | head_pitch | [-1.60, +1.60] rad clear; lower: head / trunk: lower shell / body at -1.20 rad (pose-dependent) |
| PASS | head_yaw | [-2.50, +2.50] rad clear |
| PASS | head_roll | [-0.24, +0.24] rad clear |
| PASS | beak | [+0.00, +0.40] rad clear |

## poses

| status | item | detail |
|---|---|---|
| PASS | stand (microduck stance): balance | CoM 1.7 cm inside the feet |
| PASS | stand (microduck stance): within limits | yes |
| PASS | stand (microduck stance): collisions | none |
| PASS | crouch: balance | CoM 1.7 cm inside the feet |
| PASS | crouch: within limits | yes |
| PASS | crouch: collisions | none |
| PASS | deep squat: balance | CoM 1.7 cm inside the feet |
| PASS | deep squat: within limits | yes |
| PASS | deep squat: collisions | none |
| WARN | single-leg stance (walking): balance | CoM 2.9 cm outside the stance foot: a dynamic gait, no static single-leg balance (no ankle roll) |
| PASS | single-leg stance (walking): within limits | yes |
| PASS | single-leg stance (walking): collisions | none |
| PASS | floor pick: balance | CoM 1.7 cm inside the feet |
| PASS | floor pick: within limits | yes |
| PASS | floor pick: collisions | none |
| PASS | head tucked in: balance | CoM 1.7 cm inside the feet |
| PASS | head tucked in: within limits | yes |
| PASS | head tucked in: collisions | none |
| PASS | head forward and down, beak open: balance | CoM 1.7 cm inside the feet |
| PASS | head forward and down, beak open: within limits | yes |
| PASS | head forward and down, beak open: collisions | none |

## torque

| status | item | detail |
|---|---|---|
| PASS | neck_pitch | 0.23 N m in 'head forward and down, beak open' vs rated 0.31 N m (1 x XL330_M288): 74 % used |
| PASS | head_pitch | 0.16 N m in 'head forward and down, beak open' vs rated 0.31 N m (1 x XL330_M288): 53 % used |
| PASS | L_knee | 0.16 N m in 'crouch' vs rated 0.31 N m (1 x XL330_M288): 52 % used |
| PASS | R_knee | 0.16 N m in 'crouch' vs rated 0.31 N m (1 x XL330_M288): 52 % used |
| PASS | L_hip_roll | 0.20 N m x 1.5 in 'single-leg stance (walking)' vs peak 0.96 N m (1 x XL330_M288): 31 % used |
| PASS | L_hip_pitch | 0.15 N m x 1.5 in 'floor pick' vs peak 0.96 N m (1 x XL330_M288): 23 % used |
| PASS | R_hip_pitch | 0.15 N m x 1.5 in 'floor pick' vs peak 0.96 N m (1 x XL330_M288): 23 % used |
| PASS | R_hip_roll | 0.06 N m in 'crouch' vs rated 0.31 N m (1 x XL330_M288): 20 % used |
| PASS | L_ankle_pitch | 0.04 N m in 'head forward and down, beak open' vs rated 0.31 N m (1 x XL330_M288): 13 % used |
| PASS | R_ankle_pitch | 0.04 N m in 'head forward and down, beak open' vs rated 0.31 N m (1 x XL330_M288): 13 % used |
| PASS | R_hip_yaw | 0.04 N m in 'head forward and down, beak open' vs rated 0.31 N m (1 x XL330_M288): 11 % used |
| PASS | L_hip_yaw | 0.03 N m in 'head forward and down, beak open' vs rated 0.31 N m (1 x XL330_M288): 11 % used |
| PASS | beak | 0.01 N m in 'head tucked in' vs rated 0.31 N m (1 x XL330_M288): 4 % used |
| PASS | head_roll | 0.00 N m in 'crouch' vs rated 0.31 N m (1 x XL330_M288): 1 % used |
| PASS | head_yaw | 0.00 N m in 'head forward and down, beak open' vs rated 0.31 N m (1 x XL330_M288): 0 % used |
