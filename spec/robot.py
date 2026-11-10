"""milli, a microduck-sized biped on Feetech STS3215 legs: the one source of truth for kinematics, servos, masses
and outer shape.

Run `python -I spec/robot.py` to write spec/robot.json, which the offline checks, the robot-file writer and the
Fusion build all read. Never hand-edit robot.json or the Fusion document: change this file and rebuild.

Frame: metres, X forward, Y left, Z up, floor at z = 0. Zero pose: standing with straight legs, head level, beak
shut. All geometry is written in WORLD coordinates at the zero pose. Each link's frame sits at its parent joint's
origin with world-aligned axes. The left leg is written once; the right is its mirror (y -> -y).

The look is Pollen Robotics' microduck (2026-11-02, the user: "looks of microduck for sure"): a big rounded hood
of a head with one eye and an orange bill on a neck of bare servos, a small white body, bare black servos at the
hips and knees, orange shoes on yellow soles. Its joint layout too (leg: hip yaw, roll, pitch, knee, ankle; head:
neck pitch, head pitch, head yaw, head roll, beak). Its servos are not: the hips and knees are Feetech STS3215s,
the SO-101 arm's servo, ankles too; the neck and head ROBOTIS XC330/XL330s (spec/actuators.json). The servo cases are the
structure: each sits in its parent link with its horn on the joint axis; printed plates join horn to case.

Geometry roles (as echo's, the repo this method is forked from):
  shell     outer cover (printed PLA). May overlap the shells and parts of ADJACENT links.
  decal     cosmetic surface detail (the eye). Must touch its link.
  struct    rigid internal part (printed bracket, battery, boards). Must not overlap rigid parts of any other link.
  actuator  bought servo case. Rigid. Lives in the joint's PARENT link, its shaft on the joint axis.
"""
import copy
import glob
import json
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))
ACT = {k: v for k, v in json.load(open(os.path.join(HERE, 'actuators.json'), encoding='utf-8')).items()
       if not k.startswith('_')}

# microduck's classic colourway (it ships in four)
WHITE, ORANGE, YELLOW, BLACK, DARK, GREY = '#ECEBE6', '#E8642A', '#F2B233', '#1B1C1F', '#3A3D42', '#9A9C98'
ALU, CELL, LENS, PCB = '#A8ACB0', '#2F7DE1', '#0A1420', '#1F5A36'   # (the battery bright blue: dark, it couldn't be found in a picture: the user)
STEEL = '#C9CDD2'                                  # every screw you fit: bright, to tell them from the servos' own black ones and the dark plates
AMBER = '#F0A030'                                  # the eye ring

# One design, three servo recipes, compared in simulation (2026-11-03, the user: "three recipes - all dynamixel, hybrid, all
# feetech"; "our repo can serve like a case study"). MILLI_RECIPE picks one; hybrid, the original, is the default and writes
# spec/robot.json, the others spec/robot_<recipe>.json. K: the leg's and body's plain lengths, for a smaller servo. HK: the head,
# over microduck's. Bracket and wall thicknesses don't scale, nor do parts sized by a bought part.
RECIPES = {
    # (2026-11-05, to the all-Dynamixel duck's standard. No foot switches. The Pi's USB edge up and plugged. Two buses stay two: two protocols on one wire isn't proven, and the head runs at 5 V, the legs at 2S.
    # two_boards: the same Waveshare board for both, not an FE-URT-1 and a U2D2 (which gives no power, so a splice): one on the back of the Pi's plate on USB for the Dynamixels, as the all-Dynamixel's; one on a plate at the back of the head on the Pi's UART for the Feetechs, its 2S feeding the Pi's small regulator and its own logic.
    # No board fits in the body: the most room there is 15 x 8 mm across (42 x 33 x 12.6 wanted). So the legs' current stays in the body through a splice, and only data and ground go up the neck.
    # One 8 A UBEC in the body, for the head's servos; the Pi's own USB regulator in the head, as the all-Dynamixel's, in place of a second 8 A UBEC and its soldered lead)
    'hybrid': dict(yaw='STS3215_C046', leg='STS3215_C044', neck='XC330_M288', head='XL330_M288', K=1.0, HK=1.18, shoe_horn_relief=True, seat_step_more=0.0025, foot_switch=False, pi_ports_up=True, two_boards=True, reg_head=True, reg_box=((0.0580, -0.0442, 0.2645), (0.0930, -0.0357, 0.2815)), charge_port=True,
                   port=dict(xt30=(-0.0565, -0.0405, 0.1500, 0.1615), balance=(-0.0560, -0.0470, 0.1625, 0.1695), y=(0.0268, 0.0330), cut=((-0.0620, 0.0266, 0.1496), (-0.0400, 0.0340, 0.1702))),
                   battery=dict(name='battery 2S 850 mAh', size=(0.015, 0.054, 0.030), mass=0.052), short=0.003, ranges=dict(head_roll=(-0.33, 0.33)), mount_in=0.0008,
                   mass_budget_kg=1.65,    # 1.55 before the cable plugs, real regulators and boards and the beak's drive were modelled (2026-11-03)
                   # the real bus boards put the head's weight 11 % over the head pitch XL330's rated torque, an estimate at a third of stall, in this pose: passed
                   # through, at 37 % of stall, not to be held for long. An XC330 there would hold it (0.93 N m stall) at $76 more
                   brief=['head forward and down, beak open']),
    # full micro, as microduck: XL330s at every joint, on a 2S pack straight, as microduck's are: its trainer models them at 6.5-8.2 V
    # with a 0.96 N m limit (robot/microduck/joints_properties.xml; microduck_constants.py), past ROBOTIS's 6 V maximum. xl330_2s
    # below: the head at 1.14: the beak servo and the head roll's reach don't fit in 1.0. Microduck's
    # ranges, where the parts allow them. A smaller pack, as the body has 28 mm under its plate (an estimate of a 2S 500 mAh:
    # confirm a part before ordering)
    'dynamixel': dict(plate_y=0.0235, plate_in=0.0022, yaw='XL330_M288', leg='XL330_M288', neck='XL330_M288', head='XL330_M288', xl330_2s=dict(peak_nm=0.96, rated_nm=0.31, noload_rads=16.0, volts=7.4), K=0.84, HK=1.14,
                      # a real pack on Amazon (2026-11-04, the user read its listing): OVONIC 2S 450 mAh 100C, XT30, 61.5 x 15.8 x 13.5 mm, 31 g. Across the back of the body, off-centre, so its
                      # leads have about 8 mm beside its end. Lead lengths aren't known, so the room is generous: both plugs sit in that gap, with a port in the shell's back wall
                      # (2026-11-06, the user: "slightly increase the back case cover": 13 mm more at the back, for the KCD1 rocker's 22 mm behind its bezel; the charge port's plugs move back with it)
                      battery=dict(name='battery 2S 450 mAh', size=(0.0135, 0.0615, 0.0158), mass=0.031), strap=0.0024, batt_x=-0.0410, batt_y=-0.0035, body_wide=0.0035, body_back=0.016, horn_recess=True, ankle_v2=True, roll_fork=True, charge_at=dict(port=(-0.0730, -0.0570, 0.1290, 0.1505), xt30=(-0.0615, -0.0455, 0.1300, 0.1415), bal=(-0.0610, -0.0520, 0.1425, 0.1495)), charge_port=True, head_fixes=True, reg_head=True, foot_switch=False, pi_ports_up=True, reg_box=((0.0660, -0.0450, 0.2430), (0.1010, -0.0365, 0.2600)), reg_plug=((0.0520, -0.0430, 0.2445), (0.0660, -0.0370, 0.2585)),
                      beak_y=(0.0325, 0.0355), beak_crank=(0.014, 0.65), imu_x=-0.0400, neck_open=True, cover_r=0.35, jaw_drop=0.0180, pi_back=0.0258, switch_drop=0.0015, rail_z=0.009, wire_w=0.0116,
                      ranges=dict(hip=(-1.57, 0.95), knee=(-1.1, 1.57), ankle=(-1.50, 1.535), head_roll=(-0.24, 0.24), neck_pitch=(-1.05, 1.6)), short=0.003,
                      neck_top=True, yaw_over_neck=True, xn_out=-0.005, zn_up=0.0095, slot_back=0.024, notch_z=0.003, body_screws=((-0.0382, -0.0155), (-0.0175, 0.0165)),   # (2026-11-10, the user: the neck nearer the hip servos, held by walls at their sides, the head yaw horn over the neck, and microduck's neck range: back 1.05, forward 1.6. Then (the user): no mount in front of the hip servos at all, the neck's fork plates slanting down onto the top plate over them. So the neck 9.5 mm up, as microduck's sits over its hip servos, a notch 24 mm back in the shell's top, 3 mm over the plate, the IMU 8 mm back and the shell's 2 screws to the sides, out from under the notch)
                      neck_rise=0.0495, za=0.029, collars=False, cap=0.0105, rib_up=0.004, boss_in=0.003, beak_x=0.0375, beak_z=0.018, plate_back=0.005,
                      # (2026-11-06, the user's BOM, PR 6: Amazon had no 16 x 22 x 4 mm bearings till the month's end. The uxcell 6703ZZ, 17 x 23 x 4 mm, 6.4 g (32 g for 5). Its bore is
                      # 1 mm over the 16 mm horn, so the inner race sits on the printed boss alone, on the top 2 mm, not half on the horn: test the fit on the first joint)
                      xl_bearing=dict(part='6703ZZ', d=0.017, D=0.023, mass=0.0064, shoulder_r=0.00925),   # (the boss's shoulder stays at the old 9.25 mm: 0.75 mm of the inner race, clear of its shield; at 9.75 its thin lip hung past the head roll's plate)
                      # (the same PR: no right-angle USB leads, hard to find. The regulator's 5 V soldered to the Pi's pins 2 and 6; the Pi's full UART, GPIO 14 and 15, and a ground
                      # on pin 14, to the bus board's TX, RX and GND header, its jumpers at A. No plug stands on the Pi's top edge, the board's USB-C or the regulator's socket)
                      pi_uart=True, power_switch=True, pc_port=True),
    # the micro's layout on the servo the community's Microduck builds use (2026-11-03, the user: "add it too"), rated to 8.4 V, so no running past a maker's limit. No part of any such build is used.
    # Built up from a draft to the all-Dynamixel duck's standard (2026-11-05): the same flags, for the same reasons, as the HD-1910's case is the XL330's size to the 0.1 mm (Feetech's STEP file and drawing). What differs is
    # in hd_facts, from Feetech's own spec sheet: a rear horn fitted, on a 5 mm tube that stays if it comes off; 2.0 mm plugs, which don't fit the Waveshare board's 2.5 mm sockets; no idlers to buy
    'hd1910': dict(plate_y=0.0235, plate_in=0.0022, yaw='HD1910_C001', leg='HD1910_C001', neck='HD1910_C001', head='HD1910_C001', head2='HD1910_C001', K=0.84, HK=1.14,
                   hd_facts=dict(
                       mass=0.021, peak_nm=1.47, rated_nm=0.363, noload_rads=11.8, gear=320.0, volts=7.4, stall_a=2.0, rated_a=0.9,
                       connector='AMP2.0-3P, 2.0 mm pitch, 2 sockets; 15 cm lead, all 3 wires black: pin 1 signal, 2 Vcc, 3 ground',
                       case_screw='M2 x 8 tap (bought)', horn_screw='M2 machine (the horn is drawn 8-M2.0: bought)',
                       horn_hole_deg=[0, 90, 180, 270],   # 8 in the horn, at 45 deg; the 4 on the axes, as the XL330's, so the same plates fit
                       horn_hole_depth=0.0025,            # the disc's 2.5 mm, open underneath, then 0.5 mm of air to the case
                       centre_screw=dict(r=0.00242, proud=0.0008, proud_rear=0.0004),     # M2.5 x 6, standing proud of each horn: every plate needs a hole
                       rear_stub=dict(r=0.0025, h=0.0025),   # the rear horn turns on a 5 mm tube of the case. Take the horn off and this stays
                       mount_source="FEETECH spec sheet HD-1910-C001 A/0, 2026-09-07, and STEP HD-1910-C001-20260902.stp from the FEETECH wiki (ftservo.github.io/ftservo-wiki), measured: "
                                    "case 34 x 20 x 23, horn top 3.0 off the case, shaft 7.5 off centre, frame holes 1.6 at 30 x 16, through: 4.5 mm of plastic at the rear, 2.3 at the front under a 1.2 mm counterbore; horn 16 dia with 8 holes on a 12 circle. "
                                    "At 7.4 V, +/- 10 %: 15 kg cm stall, 2 A, 113 rev/min; 3.7 kg cm rated, 0.9 A. Armature, damping and friction are still estimates, from the XL330's"),
                   battery=dict(name='battery 2S 450 mAh', size=(0.0135, 0.0615, 0.0158), mass=0.031), strap=0.0024, batt_x=-0.0410, batt_y=-0.0035, body_wide=0.0035, body_back=0.003, charge_port=True, head_fixes=True, reg_head=True, foot_switch=False, pi_ports_up=True, reg_box=((0.0660, -0.0450, 0.2430), (0.1010, -0.0365, 0.2600)), reg_plug=((0.0520, -0.0430, 0.2445), (0.0660, -0.0370, 0.2585)),
                   beak_y=(0.0325, 0.0355), beak_crank=(0.014, 0.65), imu_x=-0.0320, neck_open=True, cover_r=0.35, jaw_drop=0.0180, pi_back=0.0258, switch_drop=0.0015, rail_z=0.009, wire_w=0.0116,
                   ranges=dict(hip=(-1.57, 0.43), knee=(-1.1, 1.57), ankle=(-1.535, 1.535), head_roll=(-0.24, 0.24)), short=0.003,
                   target_tol=0.12, neck_rise=0.0495, za=0.029, collars=False, cap=0.0105, rib_up=0.004, boss_in=0.003, beak_x=0.0375, beak_z=0.018, plate_back=0.005, thigh_post_r=0.0045),
    # every joint a Feetech, on the 2S pack, no 5 V servo rail. The head and neck grow round the bigger case: Q, the head's plain internal
    # lengths over the hybrid's (the case's width over the XL330's), the hood to match
    'feetech': dict(yaw='STS3215_C046', leg='STS3215_C044', neck='STS3215_C044', head='STS3215_C044', head2='STS3215_C046', K=1.0, HK=1.46, Q=1.235,
                    neck_rise=0.074, rail=0.044, boss_in=0.003, plate_y=0.022, shell_gap=0.016, ranges=dict(head_roll=(-0.29, 0.29)),
                    # what this recipe gives up, written here rather than hidden: over the 1.55 kg budget, its neck too long to keep microduck's
                    # proportions within 10 %, and its heavy head leans the floor pick past the ankle's range
                    mass_budget_kg=1.95, target_tol=0.15, cannot=['floor pick'],
                    # one bus, one board (2026-11-05, as the all-Dynamixel's): 15 servos of one maker need neither the U2D2 nor the 5 V servo rail. The Waveshare board is Feetech's own socket
                    # (no adapter). Its power stays on the battery side for the legs: docs/feetech/WIRING.md
                    buses=1, foot_switch=False, pi_ports_up=True, reg_head=True, charge_port=True, reg_box=((0.0700, -0.0470, 0.3122), (0.1050, -0.0385, 0.3292)),
                    # the hybrid's real pack (Zeee 2S 850 mAh, 54 x 30 x 15 mm), not the estimate. Its leads come out of its +y end, so the 2 plugs sit beside it, faces at the back wall
                    battery=dict(name='battery 2S 850 mAh', size=(0.015, 0.054, 0.030), mass=0.052),
                    charge_at=dict(xt30=(-0.0555, -0.0395, 0.1500, 0.1615), bal=(-0.0550, -0.0460, 0.1625, 0.1695), port=(-0.0620, -0.0515, 0.1490, 0.1705)),
                    # the neck and head servos' plugs (they were a draft: nothing but the legs had any). An STS3215's sockets are on its rear face, 11.4-16.5 mm from the shaft, so:
                    # a rear plate 3 mm further out where a fork passes them, the room for a plug and its wires' turn (rear_stack; idlers in that plane);
                    # no rear bearing on the head roll, as its 13.5 mm race would sit on the plugs; the beak's one plug (the chain's end) on the upper socket
                    head_plugs=True, rear_stack=0.0076, neck_idler=(-0.034145, -0.031145), pitch_idler=(0.02744, 0.03044), slot_y=(-0.0365, 0.0365), neck_open=True,
                    roll_bearing=('horn',), roll_sink=(), rail_steps=True,
                    # the head's base and band open round the yaw axis, wide enough for the yaw cradle's far end, 41 mm from it, to go round (a wider slot left the frame's risers hanging)
                    yaw_round=0.0435,
                    # everything over the yaw plate up 2.2 mm, the plate that much thicker: the beak's rear plate met the yaw bearing past 0.22 rad of head roll. Raising the beak alone put its case through the hood,
                    # and a shorter boss did too: the hood curves in over it (0.17 cm3, 3.7 mm deep along its top edge)
                    yaw_plate_up=0.0022, hood_rear_x=-0.0310,
                    # the hood's back cap scaled with the head: at the small duck's 4 mm, it flared 28 to 67 mm wide so fast that Fusion refused the loft (ASM_BAD_UV_SKIN_DIR)
                    hood_tail=(0.0051, 0.0077, 0.018),
                    # (the yaw plate's +y edge in, 8.4 mm from the axis, past the horn screws' heads: the beak's rear plate passes it as the head rolls)
                    yaw_plate_y=0.0084,
                    # found by Fusion's interference check on the real bodies, past every check on the spec's shapes: the yaw plate 0.1 mm into the bearing's outer race, which turns against it; the rail without the
                    # pocket for the beak case's rear lump (36 mm3); and the shoe solid where the ankle horn's lower edge sits, 48 mm3 of plastic, so the shoe couldn't seat
                    yaw_plate_low=0.0, beak_rail_pocket=True, shoe_horn_relief=True,
                    # (the case's raised top starts about 9 mm from the shaft on the maker's STEP, 11 in 'steps': the lid's seat ring stood up to 0.95 mm in it, at 2 spots on every lid, so no lid could seat)
                    seat_step_more=0.0025,
                    # (the base's wall 1.5 mm or more at its back, where the hood narrows: it was 0.5. A rib 0.3 mm deeper on the shin: 55 N on this heavier duck put it 2 % over the beam estimate's limit, and 1 mm met the roll cradle at the knee's end)
                    wall_in=0.0375, rib_back=0.0033, frame_back=0.0003,
                    # the eye ring, a 44 mm disc 14 mm thick at this head's scale, was 25 g of solid plastic in front of the neck: a cup, open at the back, 3 mm at its front
                    eye_cup=True),
    # cheap one-way PWM hobby servos (2026-11-06, the user: "a microduck-sized duck on cheap hobby servos, driven by a PCA9685"): MG996R at the legs and neck, a DS3218
    # where the torque checks need it (R['servo'], by joint), MG90S at the head's yaw, roll and beak, where the load is about zero. Their numbers and what is estimated:
    # spec/actuators.json. Held by their ears (flange_cradle), the far side of a fork on a printed hub in a bearing (rear_hub). The hybrid's head and legs, round the
    # standard-size case: its ears reach 7 mm past each end, so the knee sits further back, the ankle's cap higher, and the two neck servos further apart
    'hobby': dict(yaw='MG996R', leg='MG996R', neck='DS3218', head='MG996R', head2='MG90S', K=1.0, HK=1.25, buses=1,
                  servo=dict(knee='DS3218'),
                  foot_switch=False, charge_port=True,
                  # the computer in the body (pi_body): the Pi, its PWM bonnet, the Pi's 5 V converter, the IMU, and two 6 V BECs in the pockets beside the yaw servos;
                  # the shell 19.5 mm taller over the plate for them, its 2 screws in front of the Pi
                  pwm=True, pi_body=True, becs=True, reg_name='6 V BEC', body_over=0.028, imu_x=-0.020, imu_y=-0.010, body_screws=((0.012, 0.020), (0.012, -0.006)),
                  battery=dict(name='battery 2S 850 mAh', size=(0.015, 0.054, 0.030), mass=0.052),
                  port=dict(xt30=(-0.0565, -0.0405, 0.1500, 0.1615), balance=(-0.0560, -0.0470, 0.1625, 0.1695), y=(0.0268, 0.0330), cut=((-0.0620, 0.0266, 0.1496), (-0.0400, 0.0340, 0.1702))),
                  port_up=0.1778935620008236,
                  # the standard-size servos' ears: the knee 5 mm higher over the ankle's far ears, the two neck servos 77 mm apart (their far ears face each other), their brackets round the ears' sweep
                  zk=0.062, neck_rise=0.0770, mount_drop=0.0235, zn_up=0.001, yaw_foot=0.024,
                  neck_idler=(-0.0375, -0.0345), pitch_idler=(0.0338, 0.0368), slot_y=(-0.0405, 0.0325), neck_open=True,   # (the slot round the neck's far bracket and hub too, and on the other side round the key's way to the neck's horn screws: the wide neck stands out of the body's front)
                  
                  # (the neck 6 mm further forward: its servo's near ears, 20 mm from its axis, swing back past the neck wall as it folds forward 1.6 rad)
                  xk_more=0.0015,    # (the knee 1.5 mm further back: the hip pitch lid's foot, on its far ears, and the knee's lid were 0.5 mm apart)
                  rib_back=0.0045, roll_arm_in=0.0025,   # (and the roll bracket's arm 1 mm thicker: 1 % over at 55 N)   # (the shin's rib 1.5 mm deeper: 55 N under the knee horn put it 11 % over the beam estimate's limit)
                  xn_out=0.006, plate_y=0.032, hood_rear_x=-0.0095, neck_plate_top=0.006,   # (the neck column's plates stop at the neck servo's top face, 6.6 mm: past it, the servo couldn't slide in)
                  # the head roll 0.25 each way (the hybrid's 0.33): the head pitch fork's bridge spans the wider neck servo, and the head's lower shell meets it past 0.26
                  ranges=dict(head_roll=(-0.25, 0.25)), short=0.0,
                  # what it gives up, declared: heavier than the hybrid, and its tall neck puts its proportions within 15 %, not 10 (as the all-Feetech duck's)
                  mass_budget_kg=1.95, target_tol=0.15),
}
# a duck at any height, from tools/new_duck.py
for _f in sorted(glob.glob(os.path.join(HERE, 'recipes', '*.json'))):
    RECIPES[os.path.splitext(os.path.basename(_f))[0]] = json.load(open(_f, encoding='utf-8'))
RECIPE = os.environ.get('MILLI_RECIPE', 'hybrid')
R = RECIPES[RECIPE]
LEG_STS = R['leg'].startswith('STS')
BUSES = R.get('buses', 2 if LEG_STS else 1)         # a Feetech chain and a Dynamixel chain, or one chain: what the boards are
K = R['K']
Q = R.get('Q', 1.0)
SHORT = R.get('short', 0.0)                         # a case this much shorter than the XL330's along its shaft: the plates at its rear face move in
H2 = R.get('head2', 'XL330_M288')                   # the head's yaw, roll and beak servos
if 'xl330_2s' in R:
    # stall from microduck's identification; rated and no-load speed scaled from ROBOTIS's 5 V numbers (rated by the stall's ratio, speed by
    # the voltage's): estimates, to measure on a real servo. Running past 6 V is microduck's choice: expect less life than ROBOTIS rates
    ACT['XL330_M288'] = dict(ACT['XL330_M288'], **R['xl330_2s'])
if 'hd_facts' in R:
    ACT['HD1910_C001'] = dict(ACT['HD1910_C001'], **R['hd_facts'])
RANGE = dict(dict(hip=(-1.3, 0.43), knee=(-1.1, 1.05), ankle=(-1.3, 1.3), head_roll=(-0.44, 0.44)), **R.get('ranges', {}))

LINKS, JOINTS = {}, []

# Materials (tools/mass_model.py): a part's mass is its material's density times its volume (a shell's wall, a
# bracket's solid times a fill factor by thickness), unless the part gives an explicit 'mass'.
MATERIALS = {
    'pla': dict(density=1240, note='PLA (FDM): shells, brackets and shoes; brackets with a fill factor'),
    'tpu': dict(density=1210, note='TPU 95A: the soles'),
    'cells': dict(density=2500, note='battery: explicit mass'),
    'electronics': dict(density=1500, note='computer, servo hat, regulator, IMU, camera: explicit masses'),
    'steel': dict(density=7800, note='bought ball bearings: explicit masses, half on each race'),
}


def material_of(g):
    if g.get('material'):
        return g['material']
    if g['role'] == 'actuator':
        return 'actuator'
    return 'tpu' if g['color'] == YELLOW else 'pla'


# ---------------------------------------------------------------- primitives (world coords, zero pose)

def cyl(name, c, axis, r, h, color=ALU, role='struct'):
    return dict(kind='cyl', name=name, c=list(c), axis=axis, r=r, h=h, color=color, role=role)


def ell(name, c, r, color=GREY, role='shell'):
    return dict(kind='ellipsoid', name=name, c=list(c), r=list(r), color=color, role=role)


def rbox(name, c, size, fillet, color=ALU, role='struct', exposed=False, **extra):
    """extra: mass-model fields ('mass', 'add_mass', 'add_what', 'material', 'fill'; tools/mass_model.py)."""
    return dict(kind='rbox', name=name, c=list(c), size=list(size), fillet=fillet, color=color, role=role,
                exposed=exposed, **extra)


def cap(name, p0, p1, r, color=BLACK, role='shell'):
    return dict(kind='capsule', name=name, p0=list(p0), p1=list(p1), r=r, color=color, role=role)


def loft(name, sections, color=GREY, role='shell', open=('top', 'bottom'), clip=None, keep=None, along='z'):
    """sections: (z, cx, cy, a, b, n): superellipse |dx/a|^n + |dy/b|^n = 1 in the plane z, bottom to top or
    top to bottom. n = 2 is an ellipse, larger n is squarer. open: ends where the skin is a sleeve (no cap).
    clip: (point, normal): keep only the part in front of this plane (a visor cut from a head shape); or a list of
    them: keep the part in front of every one (a shoe collar, a band between two planes).
    keep: (centre, radius): keep only the part inside this sphere (a domed top, round about the vertical through
    the centre, so a yaw joint on that line turns it in place)."""
    secs = [dict(z=z, cx=cx, cy=cy, a=a, b=b, n=n) for z, cx, cy, a, b, n in sections]
    g = dict(kind='loft', name=name, sections=sorted(secs, key=lambda s: s['z']), color=color, role=role,
             open=list(open))
    if along == 'x':          # sections are front-view slices stacked back to front: (x, cz, cy, a (half height), b, n)
        g['along'] = 'x'
    if clip:
        planes = [dict(p=list(c[0]), n=list(c[1]), **({'r': c[2]} if len(c) > 2 else {}))
                  for c in (clip if isinstance(clip, list) else [clip])]
        g['clip'] = planes if isinstance(clip, list) else planes[0]
    if keep:
        g['keep'] = dict(c=list(keep[0]), r=keep[1])
    return g


def rounded(plane, r):
    """A clip plane whose cut rim is rounded with radius r (in Fusion; the checks keep the sharp, larger shape)."""
    return plane[0], plane[1], r


def cut(g, *targets):
    """g removed from the named geoms of the same link (a window through a shoe): role 'cut', no body of its own."""
    return dict(g, role='cut', targets=list(targets))


def below(p, q):
    """Clip plane through the side-view points p = (x, z) and q, keeping what is below the line pq (any y)."""
    dx, dz = q[0] - p[0], q[1] - p[1]
    n = (dz, 0.0, -dx) if dx > 0 else (-dz, 0.0, dx)        # the normal pointing down
    return (p[0], 0.0, p[1]), n


def above(p, q):
    pt, n = below(p, q)
    return pt, tuple(-v for v in n)


def ahead(x):
    """Clip plane keeping x > x0 (in front)."""
    return (x, 0.0, 0.0), (1.0, 0.0, 0.0)


def behind(x):
    return (x, 0.0, 0.0), (-1.0, 0.0, 0.0)


LENS = '#0A1420'


def front_x(sections, y, z):
    """x of a loft's front surface at (y, z), interpolated between sections the way the loft is (PCHIP; a straight
    line between sections left the face mark buried 2-3 mm in Fusion, 2026-10-31)."""
    from scipy.interpolate import PchipInterpolator
    s = sorted(sections)
    f = [PchipInterpolator([q[0] for q in s], [q[k] for q in s]) for k in (1, 2, 3, 4, 5)]
    cx, cy, a, b, n = (float(g(z)) for g in f)
    return cx + a * (1 - abs((y - cy) / b) ** n) ** (1 / n)


def camera_port(name, sections, z, r):
    """A camera port on the centre line of a loft's front, square to the surface (the face slopes): dark ring
    0.6 mm proud, glossy lens 1.5 mm proud."""
    xs = front_x(sections, 0, z)
    slope = (front_x(sections, 0, z + 0.002) - front_x(sections, 0, z - 0.002)) / 0.004
    n = (1.0 / (1 + slope * slope) ** 0.5, 0.0, -slope / (1 + slope * slope) ** 0.5)     # outward normal
    at = lambda d: (xs + n[0] * d, 0.0, z + n[2] * d)
    return [cyl(f'{name} ring', at(0.0006 - 0.003), list(n), r, 0.006, '#2A2C30', 'decal'),
            cyl(f'{name} lens', at(0.0015 - 0.003), list(n), 0.5 * r, 0.006, LENS, 'decal')]


def revolve(name, axis, c, profile, color=GREY, role='shell'):
    """Solid of revolution about the axis-aligned line through c; profile (t, r) with t the coordinate along the
    axis (first and last r = 0). Round about its axis, so it never sweeps as a joint on that axis turns."""
    return dict(kind='revolve', name=name, axis=axis, c=list(c), profile=[list(p) for p in profile], color=color,
                role=role)


def prism(name, x0, t, pts, color, role='decal'):
    """A flat convex polygon in the plane x = x0, points (y, z), extruded t along +x."""
    return dict(kind='prism', name=name, x0=x0, t=t, pts=[list(p) for p in pts], color=color, role=role)




def plate(name, y0, t, circles, color=WHITE, role='shell', n=24, axis='y', **extra):
    """A flat plate in the side view, from y0 to y0 + t: the convex hull of circles ((x, z, r), ...) -- a rounded
    triangle round two pivots and a corner, as microduck's thigh covers and jaw tabs. axis 'x': circles (y, z, r),
    from x = y0; axis 'z': circles (x, y, r), from z = y0."""
    import math as _m
    pts = [(x + r * _m.cos(2 * _m.pi * k / n), z + r * _m.sin(2 * _m.pi * k / n)) for x, z, r in circles
           for k in range(n)]
    pts = sorted(set(pts))
    lower, upper = [], []
    cross = lambda o, a, b: (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    hull = lower[:-1] + upper[:-1]
    if axis != 'y':
        extra['axis'] = axis
    return dict(kind='plate', name=name, y0=y0, t=t, pts=[list(p) for p in hull], color=color, role=role, **extra)


def link(name, budget, geoms, note=''):
    """budget: a hand estimate of the link's mass (kg), kept for comparison; the real mass comes from the parts'
    materials (tools/mass_model.py)."""
    LINKS[name] = dict(name=name, budget=budget, geoms=list(geoms), note=note)


# The bearings, as microduck's (2026-11-02, the user: "add the bearings like micro"; measured by slicing its model).
# A thin bearing on the joint's axis, its bore round the servo's horn: the inner race half on the horn and half on a boss
# of the horn's link, which shoulders on it; the outer race on a thin floor over the case's face, so a tilt goes straight
# into the case instead of bending the servo's shaft. Round about the axis, so the turning case never meets it: no post
# round the case (a post has to clear the case's corners, 24 mm out on an STS3215, whatever the range). The bore is the
# horn's size: STS3215 20 mm (SO-101's mesh), XL/XC330 16 mm (microduck's). floor + w + clear: the case's face to the
# horn link's plate. Unlike microduck's, the outer race also sits in a printed seat. The STS3215's face steps down 2.5 mm
# round its horn ('steps' in spec/actuators.json): the seat's collar goes down that step, under the bearing, and takes 2 M2 x 6
# in the near holes. The boss makes the horn screws M3 x 10. Masses: 6704ZZ 6.9 g (EZO
# F6704ZZ listing; others list 5-10 g), the others scaled from it by volume: confirm before ordering.
BEARINGS = {
    'STS3215': dict(part='6704ZZ', d=0.020, D=0.027, w=0.004, floor=0.0006, clear=0.0004, wall=0.002, mass=0.0069),
    'XL330': dict(part='MR2216ZZ', d=0.016, D=0.022, w=0.004, floor=0.001, clear=0.0005, wall=0.0015, mass=0.0048),
    'ankle': dict(part='6700ZZ', d=0.010, D=0.015, w=0.004, mass=0.0026),
    # the hobby servos' (2026-11-06): the same bearings, on a thicker floor, as their horns stand 6.2 mm off the case (spec/actuators.json): the race round the horn's disc, not its hub
    'pwm_std': dict(part='6704ZZ', d=0.020, D=0.027, w=0.004, floor=0.0020, clear=0.0004, wall=0.002, mass=0.0069),
    'pwm_micro': dict(part='MR2216ZZ', d=0.016, D=0.022, w=0.004, floor=0.0019, clear=0.0005, wall=0.0015, mass=0.0048),
}
BEARINGS['XL330'] = dict(BEARINGS['XL330'], **R.get('xl_bearing', {}))
RACE = 0.00125                                     # a race's radial thickness as drawn (the shield between them)


def flanged(act):
    """A hobby servo: held by the 4 ears of its case, no screw holes in its faces, no rear shaft."""
    return 'flange' in ACT[act]


def bearing_of(act):
    if flanged(act):
        return BEARINGS['pwm_std' if ACT[act]['bearing'] == '6704ZZ' else 'pwm_micro']
    return BEARINGS['STS3215' if act.startswith('STS') else 'XL330']


def stack_of(act, bearing):
    """The case's face to the horn link's plate."""
    b = bearing_of(act)
    return b['floor'] + b['w'] + b['clear'] if bearing else ACT[act]['horn_h']


def joint(name, parent, child, origin, axis, lim, act, out=+1, long='-x', exposed=True, remote_at=None,
          case_in='parent', fork_at=None, fork_r=None, post=True, fork=True, sides=(1, -1), faces=('horn', 'rear'),
          bearing=(), face_t=None, stack=None, tray_full=False, sink=(), near=False, rear_stack=None, seat_wall=None):
    """A servo-driven joint. origin: the centre of the horn's outer face, on the axis. act: the servo model, its
    case in `parent`: the shaft along `axis`, the horn facing `out` (+1 / -1 along the axis) toward the child, the
    case's long side running `long` ('+x' .. '-z') from the shaft (the shaft sits shaft_x off the case's middle).
    The horn (with a 0.5 mm overlap into the child's bracket) goes on the child. act_c: the shaft at the case's
    mid-depth, on the axis (tools/checks.py 'axis'). remote_at: the servo's horn face somewhere else (a parallel
    axis): it drives the joint through a rod, as microduck's jaw; no horn on the child. The mount (mount_geoms):
    fork_at (deg) / fork_r (m) fix where the fork's post goes; post=False when the link's own plates join the fork's
    two plates; fork=False: the driven horn only, no rear horn, its screws in the link's own plate. sides: which of the
    cradle's side walls (+1 / -1 along the servo frame's w) to build; faces: which face plates; bearing: on which faces a
    bearing sits ('horn', 'rear'; BEARINGS); face_t: a thinner horn face plate than MT; sink: on which faces the horn screws' heads sink 1.6 mm into the plate, as
    another servo sits right against it (found 2026-11-03: their heads stood inside it, as Fusion's interference check showed); stack: the case's face to the plate on its horn,
    when not the usual; tray_full: the tray's rear wall runs the case's whole length, to reach the plate at its near
    end (the fit checks counted a tray hanging from a servo case as joined; no print would be); near: an XL/XC330's
    near end screwed down too, through its rear frame holes (its horn-side ones are under the bearing). tools/assembly.py ENDS."""
    # a recipe can put another servo on one joint (the hobby duck: a DS3218 where the MG996R's torque falls short): R['servo'], by joint name without its side
    act = R.get('servo', {}).get(name[2:] if name[:2] in ('L_', 'R_') else name, act)
    if flanged(act) and fork and not remote_at and 'rear' not in bearing:
        bearing = tuple(bearing) + ('rear',)       # no rear shaft: the fork's far side turns on a printed hub in a bearing (mount_geoms)
    a = ACT[act]
    ax = {'x': 0, 'y': 1, 'z': 2}[axis]
    act_c = list(remote_at or origin)
    stack = stack or stack_of(act, bearing)
    act_c[ax] -= out * (stack + a['box'][2] / 2)
    j = dict(name=name, parent=parent, child=child, origin=list(origin), axis=axis, lower=lim[0], upper=lim[1],
             act=act, act_c=act_c, out=out, long=long, exposed=exposed, remote=bool(remote_at), case_in=case_in,
             fork_at=fork_at, fork_r=fork_r, post=post, fork=fork, sides=list(sides), faces=list(faces),
             bearing=list(bearing), stack=stack, face_t=face_t, tray_full=tray_full, sink=list(sink), near=near, **({'rear_stack': rear_stack} if rear_stack else {}), **({'seat_wall': seat_wall} if seat_wall else {}),
             capacity=dict(model=act, count=1, ratio=1.0))
    JOINTS.append(j)
    return j


def servo_geoms(j, color=BLACK):
    """The servo case (in the parent) and its horn (in the child) for joint j."""
    a = ACT[j['act']]
    L, W, H = a['box']
    ax = {'x': 0, 'y': 1, 'z': 2}[j['axis']]
    lx = {'x': 0, 'y': 1, 'z': 2}[j['long'][1]]
    ls = 1 if j['long'][0] == '+' else -1
    c = list(j['act_c'])
    c[lx] += ls * a['shaft_x']
    size = [W, W, W]
    size[ax], size[lx] = H, L
    # (corners 0.3 mm: the XL330's real case is nearly square. Round 1.5 mm ones gave every pocket round corners, which the real model's interference check found: no servo would have seated)
    case = rbox(f"{j['name']} actuator (buy) {j['act']}", c, size, 0.0003, color, 'actuator')
    if 'steps' in a:                               # the case's real shape, not its bounding box
        f = servo_frame(j)
        bx = []
        for (u0, u1), wh, (a0, a1) in a['steps']:
            p_, q_ = at(f, u0, -(wh or W / 2), a0), at(f, u1, wh or W / 2, a1)
            bx.append([[min(m, n) for m, n in zip(p_, q_)], [max(m, n) for m, n in zip(p_, q_)]])
        case = dict(case, kind='stepbox', boxes=bx)
    if 'flange' in a:                              # a hobby servo's 4 ears, a plate across the case at 'under' from its middle, past both ends
        f, fl = servo_frame(j), a['flange']
        p_, q_ = at(f, -fl['span'] / 2, -W / 2, fl['under']), at(f, fl['span'] / 2, W / 2, fl['under'] + fl['t'])
        hb = [c[k] - size[k] / 2 for k in range(3)], [c[k] + size[k] / 2 for k in range(3)]
        case = dict(case, kind='stepbox', boxes=[[list(hb[0]), list(hb[1])], [[min(m, n) for m, n in zip(p_, q_)], [max(m, n) for m, n in zip(p_, q_)]]])
    if 'rear_stub' in a and 'ankle' not in j['name'] and (j['remote'] or not j.get('fork', True)):   # an HD-1910 with its rear horn off (an ankle keeps its, in its rear boss): the tube the horn turned on stays, 2.5 mm proud of the rear face, and whatever lies there needs a hole for it
        f = servo_frame(j)
        st = a['rear_stub']
        p_, q_ = at(f, -a['shaft_x'] - st['r'], -st['r'], -H / 2 - st['h']), at(f, -a['shaft_x'] + st['r'], st['r'], -H / 2)
        hb = [c[k] - size[k] / 2 for k in range(3)], [c[k] + size[k] / 2 for k in range(3)]
        case = dict(case, kind='stepbox', boxes=[[list(hb[0]), list(hb[1])], [[min(m, n) for m, n in zip(p_, q_)], [max(m, n) for m, n in zip(p_, q_)]]])
    # the horn as it really is: its top 'horn_top' off the case, whatever the plate's height. Where the plate is higher, a printed boss comes down onto it
    # (mount_geoms). Drawn tall enough to reach the plate, it fooled THREAD and FIT for two whole builds: the user asked what held the servo on this side
    real = a.get('horn_top', j['stack'])
    h_ = real + 0.0002 if j['stack'] > real + 1e-9 else j['stack'] + 0.0005
    hc = list(j['origin'])
    hc[ax] -= j['out'] * (j['stack'] - h_ / 2)
    horn = cyl(f"{j['name']} flange", hc, j['axis'], a['horn_r'] - 0.0001 if 'flange' in a else 0.4 * W, h_, ALU, 'struct')
    return case, horn


def real_horn_hole(j):
    return 'horn_top' in ACT[j['act']]


def screw_mass(f):
    """kg, steel at 7.85 g/cm3: the shank (a thread is about 0.8 of its full section) and the head."""
    d = float(f['size'].split()[0][1:]) / 1000
    head_r, head_h = d * 0.95, d * 0.75
    return 7850 * (math.pi * (d / 2) ** 2 * f['L'] * 0.8 + math.pi * head_r ** 2 * head_h)


def fast(g, size, L, p, d, **extra):
    """g, a bought screw: size, length L, p under its head, d the way it goes in (tools/assembly.py checks them)."""
    return dict(g, color=STEEL, fast=dict(size=size, L=L, p=[float(v) for v in p], d=[float(v) for v in d], **extra))


def box(name, lo, hi, color=DARK, role='struct', fillet=0.001, **extra):
    """An rbox from its corners."""
    c = [(a + b) / 2 for a, b in zip(lo, hi)]
    s = [b - a for a, b in zip(lo, hi)]
    # a printed part's boxes are square: rounded one by one, every join in the part was a V-groove, a notch to crack from, that looked like many small pieces (2026-11-04, the user).
    # The finished part is rounded instead, before its pockets are cut: cad/fusion/build.py soften(). A bought part keeps its corners
    if color != PCB and not extra.get('material'):
        fillet = min(fillet, 0.0001)
    return rbox(name, c, s, min(fillet, min(s) / 2 - 1e-4), color, role, **extra)


# ---------------------------------------------------------------- servo mounts (the LeRobot SO-101's way)
# (2026-11-02, the user: "refer to lerobot assembly guide so101"; "skip the sleeve"). Every servo slides into a
# CRADLE on the link that carries its case: a plate on each face over the case's far end, side walls, and an end wall
# with a window for the 3-pin cables; it takes 2 screws per face at the far pilot holes (STS3215: M2 x 6 self-tapping;
# XL/XC330: M2 x 8 through the case's frame holes). The other link is a FORK on both horns (the driven horn and the
# passive rear horn, 4 screws each; a centre screw on the driven horn only), its bridge round the case's short end,
# outside the case's reach. Hole positions: spec/actuators.json (measured on the makers' STEP files and meshes).
MT = 0.003                                         # printed plate thickness
# An STS3215's two 3-pin sockets are in its REAR face, 12-16 mm from the shaft toward the far end, side by side (2026-11-03,
# depth views of SO-101's mesh), and their plugs go in along the shaft. A plug stands 7 mm off that face (3.45 mm below the
# box's plane) and its wires need 3 more to turn: plates on the rear face need a window there, and whatever is behind needs
# room. Until now every check passed a thigh plate and a top plate sitting right on the sockets, with an 'end wall window' that
# no cable used. u: from the case's middle toward the far end; a: from the middle toward the horn face.
PLUG = dict(u=(-0.0011, 0.004), w=0.0092, face=-0.01405, h=0.010, mass=0.002)
PLUG_WINDOW = 0.0006                               # round the plugs, through a plate
# An XL330's sockets are in notches at the rear face's long edges: the plug stays inside the case, and what needs room is its wires, going out sideways
# (ROBOTIS's own model; spec/actuators.json). u, w, a as above; each side has one
XL_WIRES = dict(u=(-0.0035, 0.0065), w=(0.0100, 0.0125), a=(-0.0115, -0.0084), mass=0.0005)
XL_GROOVES = {'head roll': [(-1, 'head yaw plate')]}
XL_GROOVES_ALL = XL_GROOVES
# In a small thigh the cover's wall is 1 mm off the case's sides: grooves for sideways wires came right through it. These wires go out the back
# instead, through windows in the thigh plate, into the gap under the cover
XL_REAR = ('hip pitch', 'knee')

def servo_frame(j):
    """(c, u, w, a): the case's middle, and unit axes: u along the case from the shaft to its far end, a from the
    case toward the horn face, w the third."""
    a = ACT[j['act']]
    ax = {'x': 0, 'y': 1, 'z': 2}[j['axis']]
    lx = {'x': 0, 'y': 1, 'z': 2}[j['long'][1]]
    wx = 3 - ax - lx
    e = lambda i, sg: [sg if k == i else 0.0 for k in range(3)]
    c = list(j['act_c'])
    c[lx] += (1 if j['long'][0] == '+' else -1) * a['shaft_x']
    return c, e(lx, 1 if j['long'][0] == '+' else -1), e(wx, 1.0), e(ax, j['out'])


def at(f, u, w, a):
    c, U, W_, A_ = f
    return [c[k] + u * U[k] + w * W_[k] + a * A_[k] for k in range(3)]


def lbox(name, f, ur, wr, ar, color=DARK, role='struct', **extra):
    """A box given by local ranges (u, w, a) in the servo frame f."""
    p, q = at(f, ur[0], wr[0], ar[0]), at(f, ur[1], wr[1], ar[1])
    return box(name, [min(x, y) for x, y in zip(p, q)], [max(x, y) for x, y in zip(p, q)], color, role, **extra)


def lcyl(name, f, u, w, a, r, h, color, role):
    ax = 'xyz'[[abs(v) for v in f[3]].index(1.0)]
    return cyl(name, at(f, u, w, a), ax, r, h, color, role)


def rot(v, k, q):
    """v rotated by q about the unit vector k (Rodrigues)."""
    c, s_ = math.cos(q), math.sin(q)
    kv = sum(a * b for a, b in zip(k, v))
    kx = [k[1] * v[2] - k[2] * v[1], k[2] * v[0] - k[0] * v[2], k[0] * v[1] - k[1] * v[0]]
    return [v[i] * c + kx[i] * s_ + k[i] * kv * (1 - c) for i in range(3)]


def post_r(A):
    return 0.005 if 'case_holes_horn' in A else 0.004


def fork_post(j, u0):
    """(psi, rho): where the fork's post goes round the shaft in the servo frame (psi from +u toward +w), at the
    smallest distance where it clears the case and its cradle at every angle of the joint's range (GAP between).
    The case's link turns against the fork's: by -q when the case is in the parent, +q when in the child.
    j['fork_at'] (degrees) fixes psi, for a joint whose nearest clear direction is taken by its neighbours."""
    if j.get('fork_r') is not None:
        return math.radians(j['fork_at']), j['fork_r']
    A = ACT[j['act']]
    L, W, H = A['box']
    sx = A['shaft_x']
    pr = post_r(A)
    _, U, Wv, _ = servo_frame(j)
    k = [0.0, 0.0, 0.0]
    k['xyz'.index(j['axis'])] = 1.0
    sg = 1 if j['case_in'] == 'child' else -1
    rects = [(-(L / 2 - sx), L / 2 + sx, W / 2), (u0 + sx, L / 2 + sx + MT, W / 2 + MT)]   # (u from shaft), |w|
    if 'flange' in A:                              # the ears and their rims, both ends, and the side walls that carry them
        e2 = A['flange']['span'] / 2 + EAR_RIM
        rects = [(-e2 + sx, -L / 2 + sx, W / 2), (-L / 2 + sx, L / 2 + sx, W / 2 + MT), (L / 2 + sx, e2 + sx, W / 2 + MT)]
    qs = [j['lower'] + (j['upper'] - j['lower']) * i / 40 for i in range(41)]

    def clear(psi, rho):
        v = [rho * (math.cos(psi) * U[i] + math.sin(psi) * Wv[i]) for i in range(3)]
        for q in qs:
            r = rot(v, k, -sg * q)                 # the post in the case's frame
            u, w = sum(a * b for a, b in zip(r, U)), sum(a * b for a, b in zip(r, Wv))
            for a0, a1, hw in rects:
                du = max(a0 - u, 0, u - a1)
                dw = max(abs(w) - hw, 0)
                if math.hypot(du, dw) < pr + GAP:
                    return False
        return True

    best = None
    for deg in ([j['fork_at']] if j.get('fork_at') is not None else range(0, 360, 3)):
        psi = math.radians(deg)
        rho = A['horn_r'] + pr
        while not clear(psi, rho) and rho < 0.08:
            rho += 0.0005
        if best is None or rho < best[1] - 1e-9:
            best = (psi, rho)
    return best


# A hobby servo (2026-11-06, the hobby recipe: MG996R, DS3218, MG90S) has no screw holes in its faces and no rear shaft: it hangs by the 4 ears of its
# case, a plate across it near the horn end, past both ends. So its cradle is a tray (side walls, a floor on its rear face, end walls with the lead's window)
# with a RIM under each pair of ears: the far rim on the end walls, the near rim on the side walls (or on a post from the floor, where there are no sides).
# The servo drops in from the horn side; 2 screws through the near ears into the near rim; the lid sits on the case's top round the horn, with a FOOT down
# onto the far ears, and 2 longer screws through foot and ears into the far rim, which hold the lid too. The lead leaves the far end face, low: the end
# walls' window. The far side of a fork has no rear horn to screw to: a printed hub through the fork's plate, into a bearing in the tray's floor (mount_geoms).
EAR_RIM = 0.0005                                   # the rim past the ears' ends
def flange_cradle(j, f, cmt, wl, ex):
    A = ACT[j['act']]
    L, W, H = A['box']
    fl = A['flange']
    n = j['name'].replace('_', ' ')
    fc, sd = j.get('faces', ('horn', 'rear')), j.get('sides', (1, -1))
    e2, a0, a1 = fl['span'] / 2 + EAR_RIM, fl['under'], fl['under'] + fl['t']
    one = len(fl['holes']) < 4                     # one hole an ear (a micro servo): a second lid screw past the far ear's end, so the lid can't turn about one
    e2f = fl['span'] / 2 + 0.0035 if one else e2   # (the far rim and foot that much longer)
    m3 = fl['screw'] == 'M3'
    rt = 0.004 if m3 else 0.0035                   # the rims' thickness: a self-tapping screw bites 3 mm
    head = 0.0027 if m3 else 0.0019
    win = 0.006 if m3 else 0.0045                  # the lead's window (a JR lead is 3 wires, about 3.8 x 1.3 mm), low in the far end wall
    lo = -H / 2 - (cmt if 'rear' in fc else 0)
    out = [lbox(f'{n} cradle end +', f, (L / 2, L / 2 + MT), (win / 2, wl[1]), (lo, a0), **ex),
           lbox(f'{n} cradle end -', f, (L / 2, L / 2 + MT), (wl[0], -win / 2), (lo, a0), **ex),
           lbox(f'{n} cradle rim far', f, (L / 2, e2f), wl, (a0 - rt, a0), **ex),
           lbox(f'{n} cradle rim near', f, (-e2, -L / 2 - 0.0002), (-W / 2, W / 2), (a0 - rt, a0), **ex)]
    # the near rim on a post from the tray's floor, no wider than the case: what turns past the near end clears 20 mm, not 22 (side walls out to the ears' ends swept wider)
    # (over a bearing the post stops in the seat's floor, clear of the race; on the ankle it stands on the shin's stub plate, clear of the shoe's inner ear)
    p_lo = -H / 2 - 0.002 if 'ankle' in j['name'] else -H / 2 - 0.0005 if 'rear' in j.get('bearing', ()) else lo
    out.append(lbox(f'{n} cradle near post', f, (-e2, -L / 2 - 0.0002), (-W / 2, W / 2), (p_lo, a0 - rt + 0.0001), **ex))
    for sg in sd:
        out.append(lbox(f"{n} cradle side {'+' if sg > 0 else '-'}", f, (-A['shaft_x'], L / 2),   # (from the shaft out: past it, toward the near end, they swept into what the joint turns past)
                        (W / 2, W / 2 + MT) if sg > 0 else (-W / 2 - MT, -W / 2), (-H / 2, H / 2), **ex))
    lid = 'horn' in fc
    if lid:                                        # the lid's foot, on the far ears, and its cap joining it to the lid
        out += [lbox(f'{n} cradle horn face foot', f, (L / 2 + 0.0002, e2f), wl, (a1, H / 2), **ex),
                lbox(f'{n} cradle horn face cap', f, (L / 2 - 0.001, e2f), wl, (H / 2, H / 2 + cmt), **ex)]
    holes = sorted(fl['holes'], key=lambda p: -p[0]) + ([[fl['span'] / 2 + 0.00145, 0.0]] if one and lid else [])
    for k, (u, w) in enumerate(holes):
        far = u > 0
        if far and lid:                            # through the foot and the ear into the rim
            top = H / 2 + cmt
            L_ = math.floor((top - (a0 - rt) - 0.0005) * 1000) / 1000
            out.append(fast(lcyl(f'{n} cradle screw horn face {k}', f, u, w, top + 0.00075, head, 0.0015, DARK, 'decal'), fl['screw'], L_, at(f, u, w, top), [-v for v in f[3]]))
        else:                                      # through the ear alone: its head on the ear
            L_ = math.floor((a1 - (a0 - rt) - 0.0005) * 1000) / 1000
            out.append(fast(lcyl(f"{n} cradle screw rear {'far' if far else 'near'} ear {k}", f, u, w, a1 + 0.00075, head, 0.0015, DARK, 'decal'), fl['screw'], L_, at(f, u, w, a1),
                            [-v for v in f[3]], ear=True))
    return out


def rear_hub(j, f, hb, ex):
    """A hobby servo's far side, on a fork: a printed hub, pushed in through a hole in the fork's rear plate once the servo is in, turns in a bearing in the tray's floor.
    Its flange outside the plate takes 2 M2 x 4 into it. (A pin on the plate itself couldn't pass the bearing as the servo slides in between the fork's plates.)"""
    A = ACT[j['act']]
    H, sx = A['box'][2], A['shaft_x']
    b = bearing_of(j['act'])
    n = j['name'].replace('_', ' ')
    r_i, fl = b['d'] / 2 + RACE, b['floor']
    a_ = lambda x: -(H / 2 + x)                    # x out from the case's rear face
    mid = lambda x0, x1: a_((x0 + x1) / 2)
    lo = max(0.0004, fl - 0.0005)
    out = [dict(lcyl(f'{n} rear hub boss', f, -sx, 0, mid(lo, fl + b['w'] + 0.0001), b['d'] / 2 - 0.0001, fl + b['w'] + 0.0001 - lo, DARK, 'struct'), **ex),
           dict(lcyl(f'{n} rear hub plug', f, -sx, 0, mid(hb - 0.0001, hb + MT), r_i, MT + 0.0001, DARK, 'struct'), **ex),
           dict(lcyl(f'{n} rear hub collar', f, -sx, 0, mid(hb + MT, hb + MT + 0.0015), r_i + 0.0045, 0.0015, DARK, 'struct'), **ex),
           cut(lcyl(f'{n} rear hub hole', f, -sx, 0, mid(hb - 0.001, hb + MT + 0.001), r_i + 0.0002, MT + 0.002, DARK, 'struct'), f'{n} fork rear plate',
               *{'neck pitch': ['neck idler bracket'], 'head pitch': ['head pitch idler plate'], 'head roll': ['head top plate wide']}.get(n, []))]
    for t_ in {'head roll': ['head top plate wide']}.get(n, []):   # (room for the collar where the head's frame passes over it)
        out.append(cut(lcyl(f'{n} rear hub collar room', f, -sx, 0, mid(hb + MT - 0.0005, hb + MT + 0.002), r_i + 0.0048, 0.0025, DARK, 'struct'), t_))
    for k, sg in enumerate((1, -1)):
        out.append(fast(lcyl(f'{n} hub screw {k}', f, -sx, sg * (r_i + 0.0025), a_(hb + MT + 0.0015 + 0.00075), 0.0019, 0.0015, DARK, 'decal'),
                        'M2', 0.004, at(f, -sx, sg * (r_i + 0.0025), a_(hb + MT + 0.0015)), list(f[3])))
    return out


def mount_geoms(j):
    """(parts for the case's link, parts for the horn's link) for joint j."""
    A = ACT[j['act']]
    L, W, H = A['box']
    sx, hh = A['shaft_x'], j['stack']
    f = servo_frame(j)
    n, ex = j['name'].replace('_', ' '), dict(exposed=j['exposed'])
    sts = 'case_holes_horn' in A
    hob = 'flange' in A                            # a hobby servo, held by its ears (flange_cradle)
    holes_h = A['case_holes_horn'] if sts else A['flange']['holes'] if hob else A['case_holes_through']
    holes_r = A['case_holes_rear'] if sts else A['flange']['holes'] if hob else A['case_holes_through']
    # far holes: the two furthest from the shaft (the tables give them from the case's middle, the far end at -x)
    far_h = sorted(holes_h, key=lambda p: p[0])[:2]
    far_r = sorted(holes_r, key=lambda p: p[0])[:2]
    head = 0.0019                                  # M2 pan head radius
    u0 = min(-p[0] for p in far_h + far_r) - head - 0.0006
    if hob:                                        # a lid without a bearing stops short of the horn's disc
        u0 = -sx + A['horn_r'] + 0.0015
    win = 0.012 if W > 0.022 else 0.010            # cable window in the end wall
    cmt = j.get('face_t') or min(MT, hh - 0.0005)  # face plates under the horn's height (the fork turns over them);
    #                                                XL/XC330 screws sit in counterbores
    fc, sd, bg = j.get('faces', ('horn', 'rear')), j.get('sides', (1, -1)), j.get('bearing', ())
    wl = (-W / 2 - (MT if -1 in sd else 0), W / 2 + (MT if 1 in sd else 0))      # the cradle's width
    ax = j['axis']
    ai = 'xyz'.index(ax)
    d = {'x': (1, 2), 'y': (0, 2), 'z': (0, 1)}[ax]

    def pp(u, w):
        q = at(f, u, w, 0)
        return q[d[0]], q[d[1]]

    def axial(a0, a1):
        return min(at(f, 0, 0, a0)[ai], at(f, 0, 0, a1)[ai]), abs(a1 - a0)

    def ring(name, sg, a0, a1, r, bore, color=DARK, **extra):
        """A ring on the shaft, a0 to a1 from the case's face sg, and the cut that hollows it."""
        mid = sg * (H / 2 + (a0 + a1) / 2)
        return [dict(lcyl(name, f, -sx, 0, mid, r, a1 - a0, color, 'struct'), **ex, **extra),
                cut(lcyl(f'{name} bore', f, -sx, 0, mid, bore, a1 - a0 + 0.002, color, 'struct'), name)]

    def seat(nm, side, sg):
        """A face plate from the far end to a seat round the shaft, and the bearing's outer race in it."""
        b = bearing_of(j['act'])
        r_d, r_o, fl = b['d'] / 2, b['D'] / 2, b['floor']
        R, e = r_o + j.get('seat_wall', b['wall']), 0.001
        clear_in = r_d + RACE + 0.0002             # the inner race turns free of the floor
        out = [plate(nm, *axial(sg * (H / 2 - (0.00016 if sts and side == 'horn' else 0)), sg * (H / 2 + cmt)),   # on the real far top
                     [(*pp(-sx, 0), R), (*pp(L / 2 - e, wl[0] + e), e), (*pp(L / 2 - e, wl[1] - e), e)], DARK, 'struct',
                     axis=ax, **ex)]
        if cmt > fl + 1e-9:                        # a thick plate: the bore, and a floor ring under the bearing
            out += [cut(lcyl(f'{nm} bore', f, -sx, 0, sg * (H / 2 + cmt / 2), r_o - 0.00005, cmt + 0.002, DARK, 'struct'), nm)]
            a0_ = A['steps'][1][2][0] - H / 2 if sts and side == 'horn' else 0.0   # down the step round an STS3215's horn
            out += ring(f'{n} {side} seat floor', sg, a0_, fl, r_o, clear_in)
            if a0_:                                # clear of the far end's raised top
                out.append(cut(lbox(f'{n} seat floor step', f, (A['steps'][1][0][0] - RECIPES[RECIPE].get('seat_step_more', 0.0), A['steps'][1][0][0] + 0.01), (-r_o - 0.001, r_o + 0.001),
                                    (H / 2 + a0_ - 0.001, H / 2)), f'{n} {side} seat floor'))
        else:                                      # a thin sheet is the floor; a wall round the bearing on it
            out += [cut(lcyl(f'{nm} bore', f, -sx, 0, sg * (H / 2 + cmt / 2), clear_in, cmt + 0.002, DARK, 'struct'), nm)]
            out += ring(f'{n} {side} seat wall', sg, 0, fl + 0.003, R, r_o - 0.00005)
        for g_ in (1, -1):                         # no cradle wall on this side: nothing past the bearing either
            if g_ not in sd or j.get('tray_full'):  # (also past the seat: the hip arm passes close over a rolling lid's edge)
                out.append(cut(lbox(f'{nm} trim {g_}', f, (-sx - R - 0.001 if g_ not in sd else -sx + 0.009, L / 2 + 0.001),
                                    sorted((g_ * (r_o + 0.0001), g_ * (R + 0.001))),
                                    (min(sg * H / 2, sg * (H / 2 + cmt)) - 0.001, max(sg * H / 2, sg * (H / 2 + cmt)) + 0.001)), nm))
        if sts and side == 'horn':                 # the near holes, under the bearing (BEARINGS)
            for k, (x, y) in enumerate(sorted(holes_h, key=lambda p: p[0])[2:]):
                out.append(fast(lcyl(f'{n} cradle screw near {k}', f, -x, y, sg * (H / 2 + fl / 2), head, fl, DARK, 'decal'),
                                'M2', 0.006, at(f, -x, y, sg * (H / 2 + fl)), [-sg * v for v in f[3]]))
        out += ring(f'{n} {side} bearing outer race', sg, fl, fl + b['w'], r_o, r_o - RACE, ALU, material='steel',
                    mass=b['mass'] / 2, on_joint=j['name'])
        return out

    cr = []
    for side, sg in (('horn', 1), ('rear', -1)):
        if side not in fc:
            continue
        nm = f'{n} cradle {side} face'
        if side in bg:
            cr += seat(nm, side, sg)
        elif sts and side == 'rear':               # down onto the real rear face, round the case's lump
            rf = A['steps'][2][2][0]
            cr.append(lbox(nm, f, (-L / 2 - 0.0005 if j.get('tray_full') else u0, L / 2), wl, (-H / 2 - cmt, rf + 0.0001), **ex))
            cr.append(cut(lbox(f'{nm} lump', f, (min(u0 - 0.001, A['steps'][3][0][0] - 0.0005), A['steps'][3][0][1] + 0.0005), (-A['steps'][3][1] - 0.0005, A['steps'][3][1] + 0.0005),
                               (-H / 2 - cmt / 2, rf)), nm))
        elif hob and side == 'rear':               # the tray's floor, the whole case long: nothing screws into a hobby servo's faces, so the tray holds it, and with no side walls it carries the near rim's post
            u_lo = -sx + 0.0115 if 'ankle' in j['name'] else -A['flange']['span'] / 2 - EAR_RIM   # (under the near post too; the ankle's stops clear of the shoe's inner ear, and the shin's stub plate carries its post)
            cr.append(lbox(nm, f, (u_lo, L / 2), wl, (-H / 2 - cmt, -H / 2), **ex))
        else:
            # (tray_full: the rear wall the whole way up to the plate that holds it: on an STS3215 above, it was forgotten here, and the tray hung from nothing)
            cr.append(lbox(nm, f, (-L / 2 - 0.0005 if j.get('tray_full') and side == 'rear' else u0, L / 2), wl, sorted((sg * H / 2, sg * (H / 2 + cmt))), **ex))
    if hob:
        cr += flange_cradle(j, f, cmt, wl, ex)
    else:
        ea = (-H / 2 - (cmt if 'rear' in fc else 0), H / 2 + (cmt if 'horn' in fc else 0))   # the end wall's height
        cr += [lbox(f'{n} cradle end +', f, (L / 2, L / 2 + MT), (win / 2, wl[1]), ea, **ex),
               lbox(f'{n} cradle end -', f, (L / 2, L / 2 + MT), (wl[0], -win / 2), ea, **ex)]
        for sg in sd:
            cr.append(lbox(f"{n} cradle side {'+' if sg > 0 else '-'}", f, (u0, L / 2),
                           (W / 2, W / 2 + MT) if sg > 0 else (-W / 2 - MT, -W / 2), (-H / 2, H / 2), **ex))
        for side, holes, a0 in (('horn face', far_h, H / 2 + cmt), ('rear face', far_r, -H / 2 - cmt)):
            if side.split()[0] not in fc:
                continue
            sg = 1 if a0 > 0 else -1
            for k, (x, y) in enumerate(holes):
                cr.append(fast(lcyl(f'{n} cradle screw {side} {k}', f, -x, y, a0 + sg * 0.00075, head, 0.0015, DARK, 'decal'),
                               'M2', (0.006 if a0 > 0 else 0.008) if sts else 0.008, at(f, -x, y, a0), [-sg * v for v in f[3]]))
        # the near end, too: an XL330's frame holes go right through. On the horn side they're under the bearing (10.97 mm off the shaft, the bearing's radius 11), so from the rear
        # (2026-11-04, the user: "does the servo just slide inside and only gets its screws on the other side?" Yes: it was held at one end)
        if j.get('near') and not sts and 'rear' in fc or j.get('near') and not sts and j.get('near_plate'):
            a0 = -H / 2 - cmt
            for k, (x, y) in enumerate(sorted(holes_r, key=lambda p: p[0])[2:]):
                cr.append(fast(lcyl(f'{n} cradle screw rear near {k}', f, -x, y, a0 - 0.00075, head, 0.0015, DARK, 'decal'),
                               'M2', 0.008, at(f, -x, y, a0), list(f[3])))
        if sts and j.get('near') and j['name'] == 'beak' and R.get('head_plugs'):
            for k, (x, y) in enumerate(sorted(holes_r, key=lambda p: p[0])[2:]):
                cr.append(fast(lcyl(f'{n} cradle screw rear near {k}', f, -x, y, -H / 2 - cmt - 0.00075, head, 0.0015, DARK, 'decal'),
                               'M2', 0.008, at(f, -x, y, -H / 2 - cmt), list(f[3])))
    if j['remote']:
        return cr, []
    hr, hd = A['horn_hole_r'], A['horn_hole_deg']
    sh = 0.0027 if sts or hob and A['horn_screw'].startswith('M3') else head   # M3 (STS3215 horns, a hobby servo's aluminium horn) or M2 (XL/XC330 horns) pan head
    hb = j.get('rear_stack') or hh                 # the case's rear face to the plate on its rear horn, further than the horn side where a plug is to pass under it
    top, bot = H / 2 + hh, -H / 2 - hb
    fk = [] if 'horn_top' in A else [lcyl(f'{n} horn centre screw', f, -sx, 0, top + MT + 0.00075, sh, 0.0015, DARK, 'decal')]   # (it's the servo's own, under the hole in the plate)
    for side, sg in (('horn', 1), ('rear', -1)):   # the inner race on the horn and its boss, the boss's shoulder on it
        if side in bg:
            b = bearing_of(j['act'])
            fl, r_i = b['floor'], b['d'] / 2 + RACE
            fk += ring(f'{n} {side} bearing inner race', sg, fl, fl + b['w'], r_i, b['d'] / 2 - 0.00005, ALU, material='steel',
                       mass=b['mass'] / 2, on_joint=j['name'])
            fk.append(dict(lcyl(f'{n} rear hub shoulder' if hob and side == 'rear' else f'{n} {side} boss shoulder', f, -sx, 0, sg * (H / 2 + fl + b['w'] + b['clear'] / 2 + 0.0001), b.get('shoulder_r', r_i),
                                b['clear'] + 0.0002, DARK, 'struct'), **ex))
    real = A.get('horn_top', hh)
    for side, sg in (('horn', 1), ('rear', -1)):   # the boss: from the plate down onto the real horn, as the horn stops short of a plate over a bearing
        if hh <= real + 1e-9 or (side == 'rear' and not j.get('fork', True)) or (hob and side == 'rear'):   # (a hobby servo's rear: its hub, below)
            continue
        rb = bearing_of(j['act'])['d'] / 2 - 0.0001 if side in bg else A['horn_r']
        # up into the plate, or, over a bearing, under the shoulder that holds the race (a loose spacer on the head roll stops right there: tools/assembly.py)
        hi = (hb if side == 'rear' else hh) + 0.0002
        if side in bg:
            hi = bearing_of(j['act'])['floor'] + bearing_of(j['act'])['w'] + (0.0 if j['name'] == 'head_roll' else 0.0001)
            if hob:                                # (a hobby servo's horn stands over its bearing: the boss runs from the horn up into the plate)
                hi = hh + 0.0002
        mid = sg * (H / 2 + (real + hi) / 2)
        bn = f'{n} {side} seat boss'
        fk.append(dict(lcyl(bn, f, -sx, 0, mid, rb, hi - real, DARK, 'struct'), **ex))
        if 'horn_screw_top' in A and side == 'horn':   # room for the centre screw's head
            fk.append(cut(lcyl(f'{bn} relief', f, -sx, 0, sg * (H / 2 + A['horn_screw_top'] / 2 + real / 2 + 0.00015), 0.0032, A['horn_screw_top'] - real + 0.0005, DARK, 'struct'), bn))
    faces_ = [(top + MT, 1, 'horn')]
    if j.get('fork', True):
        # the fork: a plate on each horn, joined by a post where the case and its cradle never go over the joint's range
        if not hob:
            faces_.append((bot - MT, -1, 'rear'))
        fw, pr = hr + sh + 0.001, post_r(A)
        if hob:                                    # round the hub's hole, with room for its 2 screws
            fw = max(fw, bearing_of(j['act'])['d'] / 2 + RACE + 0.0055)
        if j.get('post', True) or j.get('fork_r') is not None:
            psi, rho = fork_post(j, u0)
        else:
            psi, rho = 0.0, 0.0                    # discs on the horns: the link's own plates join them
        pr = pr if j.get('post', True) else MT
        pu, pw = -sx + rho * math.cos(psi), rho * math.sin(psi)
        circ = [(*pp(-sx, 0), fw), (*pp(pu, pw), pr)]
        fk += [plate(f'{n} fork horn plate', *axial(top, top + MT), circ, DARK, 'struct', axis=ax, **ex),
               plate(f'{n} fork rear plate', *axial(bot - MT, bot), circ, DARK, 'struct', axis=ax, **ex)]
        if hob:
            fk += rear_hub(j, f, hb, ex)
        else:
          fk += [dict(lcyl(f'{n} rear horn', f, -sx, 0, -H / 2 - (min(real + 0.0002, hh + 0.0005) if hh > real + 1e-9 else hh + 0.0005) / 2, A['horn_r'], min(real + 0.0002, hh + 0.0005) if hh > real + 1e-9 else hh + 0.0005, ALU, 'struct'),
                    **ex)]
        if j.get('post', True):
            fk.append(plate(f'{n} fork post', *axial(bot - MT, top + MT), [(*pp(pu, pw), pr)], DARK, 'struct', axis=ax,
                            **ex))
    for a0, sg, nm in faces_:
        for k, dg in enumerate(hd):
            t = math.radians(dg)
            u_, w_ = -sx + hr * math.cos(t), hr * math.sin(t)
            ss = dict(clear=0.0012) if R.get('horn_recess') and not sts and not hob else None   # a 2.4 mm clearance hole, as the Koch v1.1 arm's: printed holes come out small, and a screw that threads the plate can't pull it down (2026-11-09)
            dn = 0.0016 if nm in j.get('sink', ()) else 0.0
            fk.append(fast(lcyl(f'{n} fork screw {nm} {k}', f, u_, w_, a0 - sg * dn + sg * 0.00075, sh, 0.0015, DARK, 'decal'),
                           # (an HD-1910's horn holes run out after 2.5 mm, with 0.5 mm of air to the case: an M2 x 6 through a 3 mm plate lying on its horn would drag on the case. check_horn_tips)
                           'M3' if sts or sh > 0.002 else 'M2', (0.004 if dn else 0.006) if hob else (0.012 if nm == 'rear' and j.get('rear_stack') else 0.010 if nm in bg else 0.008) if sts else 0.005 if 'centre_screw' in A and not dn and nm not in bg and hh <= A['horn_top'] + 1e-9 else (0.006 if dn else 0.008 if nm in bg or (nm == 'horn' and j.get('stack', 0) >= 0.0048) else 0.006), at(f, u_, w_, a0 - sg * dn),
                           [-sg * v for v in f[3]], **(ss or {})))
    if j['name'].endswith('hip_roll') and ROLLF:   # (held by its top end only: no cradle at the far end)
        cr = [g for g in cr if 'cradle' not in g['name']]
    return cr, fk


# ---------------------------------------------------------------- servo sizes

STS, XL = ACT[R['leg']], ACT[H2]                  # the leg's servo and the head's (named for the original)
AK = STS                                   # the ankle servo
SL, SW, SH = STS['box']                    # 45.2 x 24.7 x 35 mm (long, wide, along the shaft)
SX = STS['shaft_x']                        # shaft 12.5 mm off the case's middle
SHH = stack_of(R['leg'], True)       # the case's face to the plate on its horn, over the bearing (5 mm)
XLL, XLW, XLH = XL['box']                  # 34 x 20 x 26 mm
XLX, XLHH = XL['shaft_x'], XL['horn_h']
XLS = stack_of(H2, True)         # the same on the head's XL330s that carry a bearing (5.5 mm)
# the neck's own servo (it was the head's: the same case on every recipe until the hobby duck, whose neck is a standard-size servo and its head micro ones)
NKL, NKW, NKH = ACT[R['neck']]['box']
NKX = ACT[R['neck']]['shaft_x']
HOB = flanged(R['leg'])                    # hobby servos, held by their ears (flange_cradle)
# how far a leg servo reaches from its shaft along its length, toward its near end and its far end: its ears and their rims, on a hobby servo
NEAR = (STS['flange']['span'] / 2 + EAR_RIM if HOB else SL / 2) - SX
FAR = (STS['flange']['span'] / 2 + EAR_RIM if HOB else SL / 2) + SX
GAP = 0.003                                # clearance between parts of different links that move against each other

# ---------------------------------------------------------------- proportions (iterate here)

# Leg (round 9, 2026-11-02; the user: "something is wrong with joint positioning... get the microduck urdf and check
# joints... why so much gap between legs and main body"). microduck's own joints, measured on its model, set the
# layout; the zero pose IS microduck's stance (as its model's), so every servo case is square to the axes in it:
#   hip yaw   in the body, horn down;
#   hip roll  standing right under the yaw servo, horn forward (microduck's roll horns face front at the hip's middle);
#   hip pitch at the roll axis' height, out to the side on the roll bracket: its servo lies flat in the thigh, horn in
#             (the case rides in the thigh, as microduck's);
#   knee      behind and below the hip (the thigh slopes back 58 deg); its servo stands upright in the thigh's back;
#             one cover wraps both thigh servos (microduck's rounded triangle);
#   ankle     straight under the knee; its servo stands in the shin, between the shoe's two ears.
STANCE = 1.02                                      # microduck's thigh angle back from upright (its model: 58.4 deg)
ZA = R.get('za', 0.0295)                                        # ankle: microduck's 25 mm at 1.18; its case's lower end clears the shoe
#   knee: the STS3215 ankle's far end, its 3 mm cap and 1.5 mm to the thigh cover's 17.4 mm circle round the knee: it takes
#   up the gap under the thigh (the user, 2026-11-03), 4 mm taller than with an XC330
ZK = ZA + R.get('zk', 0.057 * K)
#   hip pitch: the knee servo just behind the hip pitch servo's back end, both in the thigh
XK = -FAR - SW / 2 - 0.0035 - R.get('xk_more', 0.0)              # -0.051: the knee behind the hip, one cradle wall (3 mm) between
#                                                    the hip pitch case's end and the knee case (2026-11-02, mounts)
COVER_R = R.get('cover_r', 0.29)                    # the thigh cover's corners, over the thigh's length: microduck's. (a small duck's servo corners came through it: 0.33)
THIGH = -XK / math.sin(STANCE)                     # 57 mm (microduck's 42 at 1.36)
ZP = ZK - XK / math.tan(STANCE)                    # 0.111
ZR = ZP                                            # hip roll: at the hip pitch axis' height, as microduck's
#   hip yaw: its horn on a plate on the roll case's top (the roll shaft 10 mm below that top): 13 mm above the roll
#            axis (microduck's: 12.5)
ZY = ZR + NEAR + 0.010 * K                      # a 10 mm spacer: room under the body for the thigh covers to roll

# Lateral (left leg, y > 0), from the middle out
YY = 0.016 * K                                         # hip yaw and roll: the two yaw cases side by side, 7 mm apart
# hip roll 2.5 mm outboard of the yaw axis: its bearing is wider than its case, and the two would meet as a leg yaws in
YR = YY + 0.0025
XRH = SH / 2 + SHH                                 # hip roll horn face, in front of the roll case (0.022)
YRS = (YR + SW / 2 + 0.0061, YR + SW / 2 + 0.0091)  # the roll bracket's side plate, clear of the rolling roll case
if HOB:                                            # (a narrower case: the roll lid's bearing seat, 15.5 mm round the axis, is then what the side plate has to clear)
    _rs = bearing_of(R['leg'])['D'] / 2 + bearing_of(R['leg'])['wall'] + GAP
    YRS = (YR + _rs, YR + _rs + 0.003)
YT_CASE = (YRS[1] + SHH, YRS[1] + SHH + SH)        # the thigh's two servo cases (0.042 .. 0.077)
YP = YRS[1]                                        # hip pitch horn face, inward onto the roll bracket
YK = YRS[1]                                        # knee horn face, inward onto the shin plate
YS = (YRS[0], YRS[1])                              # the shin plate
# the ankle case in line with the two thigh cases (the user, 2026-11-03: "why are the two servos out of alignment?"): a
# spacer on the shin plate takes up the knee's stack, so the leg reads as one column, the feet 5 mm further out
YA_CASE = (YS[1], YS[1] + AK['box'][2])            # the ankle case, outside the shin plate
# ankle horn face, outward: no bearing here, so only the real horn and its centre screw (3.05 mm on SO-101's mesh)
AK_FAR = FAR if HOB else AK['box'][0] / 2 + AK['shaft_x']   # the ankle servo's reach up the shin: over its far ears and their rim, on a hobby servo
ANKLE_STACK = ACT[R['leg']]['horn_top'] + 0.0002 if HOB else 0.0035   # (a hobby servo's horn stands 6.2 mm off its case: the shoe's ear sits on it)
RIB = R.get('cap', 0.015) * K if 'cap' not in R else R['cap']
CAP = R.get('cap', 0.015)                          # the shin cap's half width
XCHH, ANK = 0.0031, BEARINGS['ankle']              # the rear horn's disc, in the case's step (0.5 mm proud of the box)
YB = YA_CASE[0] - (0.0005 if 'steps' in AK else 0.0001 if HOB else XCHH)   # the top of the ankle servo's rear boss
YA = YA_CASE[1] + ANKLE_STACK
XP_FRONT = NEAR                             # hip pitch case's front end (0.0101; it runs back)
# microduck's shoe at 1.18 (its foot_left, sole_left and ankle_left meshes): a 13 mm yellow sole (15 here), the orange
# upper rising at the toe and heel to 22 mm, the ears round the ankle; low in the middle, where the ankle servo swings
# (the STS3215 ankle: 3 mm wider than with an XC330, and 4 mm longer to keep the shoe's shape: 68 x 52 mm)
FOOT = dict(x=(XK - 0.026 * K, XK + 0.042 * K), y=(YB - 0.00795, YA + 0.00505), sole=0.0154 * K, shoe=0.0160 * K, rim=0.0225 * K)   # sole 12.9 and upper 6 mm on microduck
ROLLF = bool(R.get('roll_fork'))                   # (2026-11-09, the user's sketch: the yaw link holds the roll servo by a cradle round its far end, down its inner side; the roll bracket forks onto its horn and idler, joined round its outer side; no roll bearing)
RC, TT = (XRH - SH / 2 - ACT[R['leg']]['horn_h'] if ROLLF else 0.0), 0.0025   # the roll servo's middle, x: its horn straight on the fork's plate, no spacer (the user, 2026-11-09); the yaw bracket's tabs, 2.5 mm, under the horn's height so the fork's plate passes over them
RH = 0.0092 if ROLLF else 0.010 * K               # the roll bracket's plates, half their height: round the fork's discs (8.9 mm), so both sides are the same rectangle (the user: one side round, one square, easy to fit wrong)
ROLL_PLATE = [(YR, ZR, RH), (YRS[1] - 0.0005, ZR - RH + 0.0005, 0.0005), (YRS[1] - 0.0005, ZR + RH - 0.0005, 0.0005)]   # both plates the same: round about the shaft, for clearance (the user), square where they meet the side
YP0, YP1 = (min(YY, YR) - SW / 2, YR + SW / 2 + (0.001 if ROLLF else 0.0))   # the yaw bracket's plate across, its tabs flush with its edges
AV = bool(R.get('ankle_v2'))                       # 2026-11-09, the user, from the first real shin and shoe: the shoe bolts straight to the servo's idler (no stub, no second bearing, no separate inner ear), the shin holds the servo on both faces, and the shoe and sole meet on flat faces
ZS0 = ZA + AK_FAR - 0.002 - 0.0037                 # how far the shin's four walls come down the servo: 3.7 mm past the far screw holes, as the Koch v1.1 arm's bracket (measured on its STL; 2026-11-09, the user: barely enough to fit the holes), so the horn stays on and the servo slides in whole, clear of its sockets
ARM, EAR_T, EAR_GAP, ZF = 0.0025, 0.0035, 0.0004, FOOT['sole'] - 0.001      # the shin's outer arm; the inner ear's thickness and its gap to the idler as the shoe slides on (Koch's forks: 0.4 mm over): an M2 x 6 goes 2.1 mm into the idler, and 2.5 mm when it has pulled the ear down onto it; the seam between shoe and sole
THIGH_Y = (YT_CASE[0] - 0.002, YT_CASE[1] + 0.0091)  # the thigh cover, wrapping both thigh servos; 3 mm wider for their plugs, which stand 6.6 mm off the cases' plane (PLUG)

# Body (round 2): the two hip yaw servos side by side; behind them the battery on edge and the Pi Zero 2 W with its
# servo hat, upright; the 5 V regulator and the IMU on the top plate. The battery is the smallest that runs it (user,
# 2026-11-02): a 2S LiPo of about 800 mAh, 60 x 30 x 14 mm, 45 g (7.4 V straight to the STS3215s; confirm a part
# before ordering). The neck servo stands bare in front of the body, as microduck's does (round 1 had it on a block
# on the body's top: 5 cm taller).
YAW_TOP = ZY + SHH + SH                            # top of the hip yaw cases (0.218)
PLATE = (YAW_TOP, YAW_TOP + 0.0035)                # the body's top plate
# round 9: the lower edge 10 mm over the yaw horns and tucked in (the hip yaw cases' feet show under it, as microduck's
# dark band), so the thigh covers can roll up under it
# round 10 (the user: "the box is also big, do we need that?"): no. The Pi Zero goes in the head, behind the face, as
# microduck's; the body holds the yaw servos, the battery and a thin servo-bus board behind it, and its front is flush
# with the neck servo's (microduck's neck column stands at its body's front edge): 98 mm long (microduck's: 96 at 1.18)
# (8.5 mm over the plate, was 4: the yaw servos' plugs come up through it, 6.6 mm over their case's plane, plus the shell's wall)
BODY = dict(x=(-0.058 * K, 0.040 * K), top=PLATE[1] + R.get('body_over', 0.0085), bottom=ZY + 0.003)   # (body_over: the hobby duck's Pi and servo bonnet stand over the plate)
BATT = R['battery']['size']
BATT_X = R.get('batt_x', -0.0535 * K)                               # the battery's back face: 15 mm thick (a real 850 mAh pack), just behind the yaw trays' end walls (-0.0381)
XN = XP_FRONT + NKW / 2 + 0.0079 + R.get('xn_out', 0.0)                                         # neck pitch axis: its case in the body's front, as microduck's
ZN = ZY + SHH - 0.001 + NKL / 2 + NKX + R.get('zn_up', 0.0)            # its case's foot level with the yaw cases' (0.207)
NECK_MD = R.get('mount_drop', 0.016 * Q)                # the neck mount's top, under the neck pitch axis: clear of what the neck servo's near end sweeps
ZH = ZN + R.get('neck_rise', 0.0514 * Q)                                   # head pitch: straight above, the neck one column (round 6)
# head yaw: its case forward of the head pitch case (clear of its 13 mm circle), so it can sit low
# head yaw (round 6, the user: "neck needs improvement"): microduck's neck is one straight column of servos, the head
# yaw servo on top of it inside the head's underside. Its case straight over the head pitch servo, clear of its 13 mm
# circle (round 5 had it 14 mm ahead: an L-shaped neck)
YAW_FOOT = ZH + R.get('yaw_foot', 0.0131 * Q + GAP + 0.004)              # the head pitch U's bridge: clear of its servo's 13 mm circle
HB = XLX if R.get('yaw_over_neck') else 0.0         # the head yaw servo back by its shaft's offset: its horn, and the head on it, straight over the neck (2026-11-10, the user)
XHY, ZHY = XN + XLX - HB, YAW_FOOT + XLH + XLS
HO = lambda p: (p[0] - HB + R.get('xn_out', 0.0), p[1], p[2] + R.get('zn_up', 0.0)) if R.get('yaw_over_neck') else p   # a head part given by its coordinates: moved with the head (with the neck, and back with the yaw servo)
YAW_UP = R.get('yaw_plate_up', 0.0)
XHR, ZHR = XHY + XLH / 2 + XLS, ZHY + XLHH + YAW_UP + XLW / 2            # head roll horn face and axis height
HEAD_Z = ZHY - 0.008 * Q + YAW_UP                               # the head's lower edge: the yaw horn 8 mm up inside it
HEAD_CX = XN + 0.006 * Q - HB                               # the head reaches as far behind the neck as in front, as microduck's
NECK_IDLER = R.get('neck_idler', (-0.0225 * Q + SHORT, -0.0190 * Q + SHORT))   # the plate on the neck servo's rear horn
PITCH_IDLER = R.get('pitch_idler', (0.0165 * Q - SHORT, 0.020 * Q - SHORT))     # the same on the head pitch servo


# ---------------------------------------------------------------- body (root)

BW_ = (R.get('body_wide', 0.0), R.get('body_back', 0.0) / 2)            # how much wider each side, and half how much further back, the body grows round a longer pack
BCX_ = -0.009 * K - BW_[1]
CHARGE_PORT = R.get('charge_at', {}).get('port', (-0.0600, -0.0440, 0.1290, 0.1505))
PORT = R.get('port') or dict(xt30=R.get('charge_at', {}).get('xt30', (-0.0485, -0.0325, 0.1300, 0.1415)), balance=R.get('charge_at', {}).get('bal', (-0.0480, -0.0390, 0.1425, 0.1495)), y=(0.0280, 0.0335), cut=((CHARGE_PORT[0], 0.0270, CHARGE_PORT[2]), (CHARGE_PORT[1], 0.0340, CHARGE_PORT[3])))
if 'port_up' in R:                                 # the hybrid's charge port, as high over its plate as on the hybrid (port_up: its plate's height there)
    _dz = PLATE[0] - R['port_up']
    PORT = dict(PORT, xt30=PORT['xt30'][:2] + tuple(v + _dz for v in PORT['xt30'][2:]), balance=PORT['balance'][:2] + tuple(v + _dz for v in PORT['balance'][2:]),
                cut=tuple((a, b, c + _dz) for a, b, c in PORT['cut']))
TWO_BOARDS = R.get('two_boards', False)             # a Waveshare board for each bus, in the head
# the hobby duck (2026-11-06, the user: "Pi, battery, IMU, BEC and PCA9685 all go in the body, so only the head's servo leads (and a camera ribbon) cross the neck"):
# its computer in the body, on 4 posts on the top plate behind the IMU, with Adafruit's 16-channel PWM / Servo Bonnet (a PCA9685, 65 x 30.4 mm, its page) on its
# GPIO header; the Pi's 5 V from a USB buck converter, the servos' 6 V from two BECs in the pockets beside the yaw servos (docs/hobby/WIRING.md)
PI_BODY = R.get('pi_body', False)
REG_NAME = R.get('reg_name', '5 V regulator')
PX_ = -0.0373                                      # the Pi's middle, its 65 mm across the body
PZ_ = PLATE[1] + 0.003                             # its board's underside, on its posts
BON_Z = PZ_ + 0.0095                               # the bonnet's underside, on an 8.5 mm stacking header over the Pi's 1 mm board

def body_computer():
    """The hobby duck's Pi Zero 2 W on 4 posts (its M2.5 holes, 58 x 23 mm), the PWM bonnet on its GPIO header, the servo plugs off the bonnet's front edge on
    right-angle headers, and the Pi's 5 V buck converter beside the IMU. Sizes: the Pi's 65 x 30 mm; the bonnet's 65 x 30.4 (Adafruit 3416's page); the converter as the
    other ducks' (35 x 17 x 8.5 mm, a listing's): confirm the parts you buy."""
    holes = [(PX_ + sx_ * 0.0115, sy_ * 0.029) for sx_, sy_ in ((1, 1), (-1, 1), (1, -1), (-1, -1))]
    out = [box('Pi Zero 2 W', (PX_ - 0.015, -0.0325, PZ_), (PX_ + 0.015, 0.0325, PZ_ + 0.0052), PCB, material='electronics', mass=0.011)]
    for k, (x_, y_) in enumerate(holes):
        out += [box(f'Pi post {k}', (x_ - 0.0025, y_ - 0.0025, PLATE[1] - 0.0002), (x_ + 0.0025, y_ + 0.0025, PZ_), fillet=0.0005),
                fast(cyl(f'Pi screw {k}', (x_, y_, PZ_ + 0.001 + 0.0009), 'z', 0.00225, 0.0018, DARK, 'decal'), 'M2.5', 0.006, (x_, y_, PZ_ + 0.001), (0, 0, -1))]
    wired = lambda n, lo, hi, m: dict(box(n, lo, hi, '#D9D4C5', fillet=0.0003), material='electronics', mass=m, wire=True, exposed=True)
    out += [dict(box('PWM bonnet', (PX_ - 0.0152, -0.0325, BON_Z), (PX_ + 0.0152, 0.0325, BON_Z + 0.0046), PCB, material='electronics', mass=0.0085)),
            dict(box('PWM bonnet header', (PX_ - 0.0130, -0.0255, PZ_ + 0.0052), (PX_ - 0.0080, 0.0255, BON_Z + 0.0001), '#202226', fillet=0.0003), material='electronics', mass=0.003),
            wired('PWM bonnet plugs', (PX_ + 0.0152, -0.0260, BON_Z + 0.0016), (PX_ + 0.0302, 0.0260, BON_Z + 0.0101), 0.004),
            wired('Pi camera ribbon', (PX_ - 0.008, 0.0325, PZ_), (PX_ + 0.008, 0.0355, PZ_ + 0.0012), 0.0002),
            wired('Pi microSD', (PX_ - 0.006, -0.0345, PZ_), (PX_ + 0.006, -0.0325, PZ_ + 0.0012), 0.0002),
            dict(box('Pi 5 V converter', (-0.0200, -0.0300, PLATE[1] + 0.0005), (0.0150, -0.0130, PLATE[1] + 0.0090), PCB, material='electronics', mass=0.008), on_cover=True)]
    return out


NSW = bool(R.get('neck_sides'))
YAW_SIDE, NSW_T = YY + SW / 2 + 0.0002, 0.0025   # the hip yaw cases' outer faces (23.4 mm), 0.2 mm clear; the walls 2.5 mm
NTOP, NTOP_R = bool(R.get('neck_top')), 0.0095   # the fork's discs round the neck's horn and idler: as the fork's own (hr + screw + 1 mm)
MOUNT_Z0 = ZN - NECK_MD - R['mount_plate'] if 'mount_plate' in R else ZY + SHH + 0.0005   # the neck mount: a plate under the neck's brackets, carried by the side walls (the user, 2026-11-10: "do we need all this extra plastic?"), or a block down to the yaw cases' feet
NSW_X0 = XP_FRONT - 0.012                # the side walls run 12 mm back along the cases' sides from their front ends


link('trunk', 0.40, [
    box('top plate', (BATT_X, -R.get('plate_y', 0.025 * K), PLATE[0]), (XP_FRONT, R.get('plate_y', 0.025 * K), PLATE[1])),
    # the wall joining the neck mount to the top plate: without it, only the hip yaw servo cases joined them
    *[box('neck wall', (XP_FRONT + 0.0001, -0.0225 * Q, ZY + SHH + 0.0005), (XP_FRONT + 0.003, 0.0165 * Q, PLATE[1]), exposed=True) for _ in range(not NSW and not NTOP)],
    *[box('neck wall tab', (XP_FRONT - 0.002, -0.0225 * Q, PLATE[0]), (XP_FRONT + 0.0002, 0.0165 * Q, PLATE[1]), exposed=True) for _ in range(not NSW and not NTOP)],
    # (neck_sides: a wall down each side of the hip yaw servos' front ends instead, from the plate to the neck mount; the user, 2026-11-10)
    *[x for sy in ((1, -1) if NSW else ()) for x in (
        box(f"{'L' if sy > 0 else 'R'} neck side wall", (NSW_X0, min(sy * YAW_SIDE, sy * (YAW_SIDE + NSW_T)), ZY + SHH + 0.0005), (XP_FRONT + 0.003, max(sy * YAW_SIDE, sy * (YAW_SIDE + NSW_T)), PLATE[1]), exposed=True),
        box(f"{'L' if sy > 0 else 'R'} neck side wall top", (NSW_X0, min(sy * (YAW_SIDE - 0.004), sy * YAW_SIDE), PLATE[0]), (XP_FRONT, max(sy * (YAW_SIDE - 0.004), sy * YAW_SIDE), PLATE[1]), exposed=True))],
    box(R['battery']['name'], (BATT_X, R.get('batt_y', 0) - BATT[1] / 2, PLATE[0] - BATT[2]), (BATT_X + BATT[0], R.get('batt_y', 0) + BATT[1] / 2, PLATE[0]), CELL,
        material='cells', mass=R['battery']['mass']),
    # the two plugs, in the gap beside the pack's end, their faces at the port: XT30 (power, and where it switches off), balance lead (charging). Sizes as commonly sold: check on the real pack.
    # 'decal' keeps them out of the offline sweeps (they ride on the pack's leads); in_cad: they're real things, so Fusion builds them, as it doesn't other decals
    *[box(nm, (x0, PORT['y'][0], z0), (x1, PORT['y'][1], z1), col, 'decal', in_cad=True) for nm, (x0, x1, z0, z1), col in (('battery XT30', PORT['xt30'], '#F2C21A'), ('battery balance plug', PORT['balance'], '#E8E8E0')) if R.get('charge_port')],
    # the two 5 V regulators stand on edge in the pockets beside the yaw servos (39 x 21 x 6 mm, an 8 A UBEC's listing); the IMU
    # on the plate, behind the yaw plugs (a BNO085 board, 25.6 x 22.7 x 4.6 mm, Adafruit's listing)
    *[box(f'{nm} 5 V regulator', (-0.0300, sy * 0.0345 - 0.003, ZY + 0.0135), (0.0090, sy * 0.0345 + 0.003, ZY + 0.0347), PCB, material='electronics', mass=0.018) for nm, sy in (('L', 1), ('R', -1)) if BUSES == 2 and (sy > 0 or not TWO_BOARDS)],
    *[box(f'{nm} {REG_NAME}', (-0.0300, sy * 0.0345 - 0.003, ZY + 0.0135), (0.0090, sy * 0.0345 + 0.003, ZY + 0.0347), PCB, material='electronics', mass=0.023) for nm, sy in (('L', 1), ('R', -1)) if R.get('becs')],   # (the hobby duck's: 18 g as the hybrid's listing, and 5 g of its side's 6 V bus spliced on)
    # (one bus and one regulator: the Pi's, on tape on the battery's back, in the 6.2 mm behind it)
    *[dict(box('L 5 V regulator', (BATT_X - 0.0060, -0.0195, PLATE[0] - 0.0215), (BATT_X, 0.0195, PLATE[0] - 0.0005), PCB, material='electronics', mass=0.018), on_cover=True) for _ in range(BUSES == 1 and not R.get('reg_head') and not PI_BODY)],
    # each hangs from 2 arms off the top plate, taped and tied (they reach past the yaw plugs, which stay clear)
    *[box(f"{nm} regulator arm {k}", (x_, min(sy * 0.0245, sy * 0.0378), PLATE[0]), (x_ + 0.008, max(sy * 0.0245, sy * 0.0378), PLATE[1]), fillet=0.0005)
      for nm, sy in (('L', 1), ('R', -1)) if BUSES == 2 and (sy > 0 or not TWO_BOARDS) or R.get('becs') for k, x_ in enumerate((-0.030, 0.001))],
    *[box(f"{nm} regulator drop {k}", (x_, min(sy * 0.0292, sy * 0.0378), ZY + 0.0345), (x_ + 0.008, max(sy * 0.0292, sy * 0.0378), PLATE[0] + 0.0002), fillet=0.0005)
      for nm, sy in (('L', 1), ('R', -1)) if BUSES == 2 and (sy > 0 or not TWO_BOARDS) or R.get('becs') for k, x_ in enumerate((-0.030, 0.001))],
    dict(box('IMU', (R.get('imu_x', -0.0426), R.get('imu_y', -0.01135), PLATE[1] + 0.0017), (R.get('imu_x', -0.0426) + 0.0256, R.get('imu_y', -0.01135) + 0.0227, PLATE[1] + 0.0063), PCB, material='electronics', mass=0.003), on_cover=True),   # (on 1.7 mm of foam tape, over 2 screw heads)
    # the legs' bus: a lead from one hip yaw servo to the other, the battery's V and G and the neck's D and G joined into it, in heat-shrink, taped to the plate between the 2 yaw plugs
    *[dict(box('leg bus splice', (-0.0100, -0.0060, PLATE[1] + 0.0003), (0.0005, 0.0060, PLATE[1] + 0.0048), '#202226', fillet=0.0005), material='electronics', mass=0.005, on_cover=True) for _ in range(TWO_BOARDS)],
    *(body_computer() if PI_BODY else []),
    # (a tray under the pack, on the yaw trays' end walls: the old strap was a solid bar through it. The shell's back wall and a hook-and-loop
    # strap round pack and tray hold it)
    box('battery strap', (BATT_X, -0.010, PLATE[0] - BATT[2] - R.get('strap', 0.003)), (BATT_X + BATT[0] + 0.0034, 0.010, PLATE[0] - BATT[2])),
    *[box(f'pack hanger {k}', (BATT_X + BATT[0], sy_ * 0.0045 - 0.0015, PLATE[0] - BATT[2] - R.get('strap', 0.003)), (BATT_X + BATT[0] + 0.002, sy_ * 0.0045 + 0.0015, PLATE[0] + 0.0002)) for k, sy_ in enumerate((1, -1)) if HOB],
    # the neck servo rides in the neck; the body holds it on both sides (round 7, the user: "one sided joints are
    # bad"): a bracket on its horn, a bracket with an idler on its other face, on a block clear of its 13 mm circle
    *[box('neck mount wings', (XP_FRONT - 0.0005, -(YAW_SIDE + NSW_T), MOUNT_Z0), (XP_FRONT + 0.003, YAW_SIDE + NSW_T, ZN - NECK_MD), DARK, exposed=True) for _ in range(NSW)],   # (under the side walls, joining them to the mount)
    *[box('neck mount', (XP_FRONT - 0.0005, -0.0225 * Q, MOUNT_Z0), (XN + NKW / 2 - R.get('mount_in', 0.0), 0.0165 * Q, ZN - NECK_MD), DARK,
        exposed=True) for _ in range(not NTOP)],
    *[box('neck bracket', (XN - NKW / 2, 0.0130 * Q, ZN - NECK_MD - 0.0005 * Q), (XN + NKW / 2, 0.0165 * Q, ZN + 0.012 * Q), DARK, exposed=True) for _ in range(not NTOP)],
    # (neck_top: the fork's two plates come down at a slant onto the top plate, over the hip yaw servos: no mount in front of them; the user, 2026-11-10)
    *[plate(f'neck top bracket {nm}', y0, 0.003, [(XN, ZN, NTOP_R), (XP_FRONT - 0.0135, PLATE[0] + 0.0015, 0.0015), (XP_FRONT - 0.0015, PLATE[0] + 0.0015, 0.0015)], DARK, 'struct', exposed=True)
      for nm, y0 in (('horn', 0.0130 * Q), ('rear', NECK_IDLER[1] - 0.003)) if NTOP],
    *[box('neck idler step', (XN - NKW / 2, NECK_IDLER[0], ZN - NECK_MD - 0.0005 * Q), (XN + NKW / 2, -0.0225 * Q + 0.0001, ZN - NECK_MD - 0.0005 * Q + 0.004), DARK, exposed=True) for _ in range(NECK_IDLER[0] < -0.0225 * Q - 1e-6 and not NTOP)],
    *[box('neck idler bracket', (XN - NKW / 2, NECK_IDLER[0], ZN - NECK_MD - 0.0005 * Q), (XN + NKW / 2, NECK_IDLER[1], ZN + 0.012 * Q), DARK, exposed=True) for _ in range(not NTOP)],
    # a superellipse of power 5 round the yaw cases' front corners (power 3.2 cut through them, round 2), as narrow
    # as microduck's body in proportion (round 5)
    loft('body', [(BODY['bottom'], BCX_, 0, 0.047 * K + BW_[1], 0.031 * K + BW_[0], 6.0), (BODY['bottom'] + 0.010 * K, BCX_, 0, 0.049 * K + BW_[1], 0.040 * K + BW_[0], 8.0),
                  (BODY['top'] - 0.008, BCX_, 0, 0.049 * K + BW_[1], 0.040 * K + BW_[0], 8.0), (BODY['top'] - 0.003, -0.010 * K - BW_[1], 0, 0.045 * K + BW_[1], 0.036 * K + BW_[0], 5.0),
                  (BODY['top'], -0.011 * K - BW_[1], 0, 0.038 * K + BW_[1], 0.028 * K + BW_[0], 3.5)], WHITE, open=('bottom',)),
    # the neck comes up out of the body through a slot in its top, as microduck's (the user, round 6)
    # down through the front too: the neck folds forward 1.6 rad to tuck the head in (Pollen's desk photo)
    *[fast(cyl(f'body screw {k}', (x_, y_, BODY['top'] + 0.0003), 'z', 0.0019, 0.0012, DARK, 'decal'), 'M2', 0.006,
           (x_, y_, BODY['top']), (0, 0, -1)) for k, (x_, y_) in enumerate(R.get('body_screws', ((-0.0455 * K, 0.0), (0.005 * K, 0.0))))],
    # (the shell's top is 8.5 mm over the plate: posts under those screws)
    *[box(f'body post {k}', (x_ - 0.003, y_ - 0.003, PLATE[1] - 0.0002), (x_ + 0.003, y_ + 0.003, BODY['top'] - 0.0014), fillet=0.0005) for k, (x_, y_) in enumerate(R.get('body_screws', ((-0.0455 * K, 0.0), (0.005 * K, 0.0))))],
    # (all-Dynamixel: the shorter body ends in front of the neck servo's front, so the slot opens to its lower edge, or the shell can't come down past it)
    cut(box('neck slot', (XP_FRONT + 0.0009, R.get('slot_y', (-0.026 * Q, 0))[0], BODY['bottom'] - 0.01 if R.get('neck_open') else ZN - 0.014 * Q), (XN + 0.040 * Q, R.get('slot_y', (0, 0.0195 * Q))[1], BODY['top'] + 0.01), WHITE, 'shell'),
        'body'),
    # (slot_back: a notch in the shell's top behind the slot, above the plate, where the neck passes as it leans back: the user, 2026-11-10, "trim the case a bit")
    *[cut(box('neck lean notch', (XP_FRONT + 0.001 - R['slot_back'], R.get('slot_y', (-0.026 * Q, 0))[0], PLATE[1] + R.get('notch_z', 0.0)), (XP_FRONT + 0.0019, R.get('slot_y', (0, 0.0195 * Q))[1], BODY['top'] + 0.01), WHITE, 'shell'), 'body') for _ in range('slot_back' in R)],
    # the charge port: the plugs' faces at the back wall, only they show (2026-11-04, the user: only the leads visible, at the body's surface)
    *[cut(box('charge port', *PORT['cut'], WHITE, 'shell'), 'body') for _ in range(bool(R.get('charge_port')))],
    # (power_switch, 2026-11-06, the user: "add a switch somewhere on the back or bottom", then "standard and easily accessible parts for anyone to buy": the XT30 sits inside the
    # port, out of a finger's reach. A standard KCD1 mini rocker, SPST (DaierTek's pre-wired 10-pack, Amazon B07S2QJKTX: hole 19.2 x 13.2 mm, bezel 21 x 15, 13.2 mm of housing and
    # 8.2 of terminals under it; 20 A 12 V DC as listed, 6 A 250 V AC on the part). The back has 8-10 mm behind its wall and the floor opens onto the left hip yaw, so the body's back grows 13 mm
    # (the user: "slightly increase the back case cover") and it sits in the back wall, right of centre, clear of the charge port: 22 mm to the battery. In the battery's red lead, its 2 leads spliced in; the XT30 stays plugged)
    *[g for _ in range(bool(R.get('power_switch'))) for g in (
        dict(box('power switch', (-0.0626, -0.0194, 0.13265), (-0.0513, -0.0006, 0.14535), DARK, material='electronics', mass=0.0031), on_cover=True),
        dict(box('power switch neck', (-0.0649, -0.0194, 0.13265), (-0.0626, -0.0006, 0.14535), DARK, material='electronics', mass=0.0004), exposed=True, on_cover=True),
        dict(box('power switch leads', (-0.0513, -0.0160, 0.1350), (-0.0431, -0.0040, 0.1430), '#D9D4C5', material='electronics', mass=0.001, wire=True), on_cover=True),
        dict(box('power switch bezel', (-0.0669, -0.0205, 0.1315), (-0.0649, 0.0005, 0.1465), DARK, material='electronics', mass=0.0005), exposed=True, on_cover=True),
        dict(box('power switch rocker', (-0.0694, -0.0175, 0.1330), (-0.0669, -0.0025, 0.1450), DARK, material='electronics', mass=0.0003), exposed=True, on_cover=True),
        cut(rbox('power switch hole', (-0.06395, -0.0100, 0.1390), (0.0026, 0.0192, 0.0132), 0.0005, WHITE, 'shell'), 'body'))],
], 'the body: hip yaw servos, battery, Pi Zero 2 W and servo hat, regulator, IMU; the neck servo in front')

# ---------------------------------------------------------------- left leg (bare servos and brackets, as microduck's)

link('L_hip_yaw_link', 0.07, [
    # (the plug window and the case's lump left a strip 0.4 mm wide between them, across the tray's rear face: no print would hold it, nor does it hold anything: WALL)
    *[cut(box('L hip roll strip', (-0.0195, 0.0087, 0.1001), (-0.0150, 0.0283, 0.1008), DARK), 'L hip roll cradle rear face') for _ in range(RECIPE == 'hybrid')],
    box('L hip yaw plate', ((RC - SH / 2 - TT) if ROLLF else -SH / 2 - (MT if not LEG_STS and not HOB else 0), YP0, ZR + NEAR), ((RC + SH / 2 + TT) if ROLLF else SH / 2, YP1, ZY), exposed=True),
    # (the roll fork: the yaw link comes down the roll servo's inner side to the cradle round its far end)
    *[box('L hip yaw inner wall', (-SH / 2 - MT, YR - SW / 2 - 0.002, ZR - AK['box'][0] / 2 - AK['shaft_x'] + 0.0040), (SH / 2 + MT, YR - SW / 2, ZR + NEAR + 0.0001), exposed=True) for _ in range(0)],   # (it was 0.5 mm over this, onto the turning outer race)
    # the near end too: a tab from the yaw plate down each face over the near screw holes, clear of the turning horn and idler (8.6 mm round the shaft), and a window for the inner plug's wires
    *[x for sg in ((1, -1) if ROLLF else ()) for yy in ((YP0, YR - 0.0035), (YR + 0.0035, YP1)) for x in (
        box(f"L hip yaw near tab {'h' if sg > 0 else 'r'}{0 if yy[0] < YR else 1}", (RC + min(sg * SH / 2, sg * (SH / 2 + TT)), yy[0], ZR + 0.005), (RC + max(sg * SH / 2, sg * (SH / 2 + TT)), yy[1], ZR + NEAR + 0.0005), exposed=True),)],
    *[cut(cyl(f'L hip yaw near tab clear {k}', (RC + sg * (SH / 2 + TT / 2), YR, ZR), 'x', 0.0086, TT + 0.002, DARK), *[f"L hip yaw near tab {'h' if sg > 0 else 'r'}{q}" for q in (0, 1)]) for k, sg in enumerate((1, -1)) if ROLLF],
    *[fast(cyl(f"L hip roll cradle screw near {'h' if sg > 0 else 'r'}{q}", (RC + sg * (SH / 2 + 0.001 + 0.00075), YR + (-0.008 if q == 0 else 0.008), ZR + 0.0075), 'x', 0.0019, 0.0015, DARK, 'decal'), 'M2', 0.008,
           (RC + sg * (SH / 2 + 0.001), YR + (-0.008 if q == 0 else 0.008), ZR + 0.0075), (-sg, 0, 0), clear=0.0012) for sg in ((1, -1) if ROLLF else ()) for q in (0, 1)],   # (the servo held by its top end only, 4 screws, as the user asked: it slides up into the tabs with its horn on)
    *[cut(box('L hip yaw wire window', (-SH / 2 - 0.0005, YR - SW / 2 - 0.004, ZR - 0.0145), (-SH / 2 + 0.0035, YR - SW / 2 + 0.0001, ZR - 0.0035), DARK), 'L hip yaw inner wall') for _ in range(0)],
])
link('L_hip_roll_link', 0.07, [
    # microduck's hip_l: from the roll horn at the front, out to the side, where the hip pitch horn turns
    *([plate('L hip roll front', XRH, 0.003, ROLL_PLATE, DARK, 'struct', axis='x', exposed=True)] if ROLLF else [box('L hip roll front', (XRH, YR - 0.011 * K, ZR - RH), (XRH + 0.003, YRS[1], ZR + RH), exposed=True)]),
    box('L hip roll side', (RC - SH / 2 - 0.0065 if ROLLF else -0.012 * K, YRS[0], ZR - RH), (XRH + MT, YRS[1], ZR + RH), exposed=True),
    *[plate('L hip roll back arm', RC - SH / 2 - 0.0065, 0.003, ROLL_PLATE, DARK, 'struct', axis='x', exposed=True) for _ in range(ROLLF)],   # (the fork's idler plate, the same outline as the horn side's, out to the side: the bridge round the roll servo's outer side)
    # (the arm from the pitch horn to the front plate carries the whole leg: 1.5 mm thicker, past the horn's screws)
    box('L hip roll arm', (0.0115 * K, YRS[0] - R.get('roll_arm_in', 0.0015), ZR - RH), (XRH + MT, YRS[1], ZR + RH), exposed=True),
    # gussets in the corner the leg's load turns through (2026-11-04, the user: "a 3d printed part like that will break so easily"; LOAD's beam estimate has no corner in it, and a sharp inside
    # corner multiplies the stress 2-3 x). Top and bottom only, outside the lid's rim as the roll case turns: its middle is taken
    *[plate(f'L roll gusset {k}', ZR + sz_ * 0.0055 * K if sz_ > 0 else ZR - 0.010 * K, 0.0045 * K,
            [(XRH + 0.0001, YRS[0] - 0.0014, 1e-5), (XRH + 0.0001, YR + math.sqrt((bearing_of(R['leg'])['D'] / 2 + bearing_of(R['leg'])['wall'] + 0.0007) ** 2 - (0.0055 * K) ** 2), 1e-5), (XRH - 0.0030, YRS[0] - 0.0014, 1e-5)],
            DARK, 'struct', axis='z', exposed=True) for k, sz_ in enumerate((1, -1))],
])
link('L_hip_pitch_link', 0.10, [
    # the thigh: the hip pitch servo lying flat (from the joint, case in the thigh) and the knee servo standing at
    # its back end, joined by a plate on their outer faces and wrapped by microduck's cover
    # the cover's outline, 3 mm in (a rectangle's corner stuck out of the cover: the user, 2026-11-02)
    plate('L thigh plate', YT_CASE[1] - 0.0002, MT + 0.0002,
          [(0.0, ZP, COVER_R * THIGH - R.get('plate_in', 0.003)), (XK, ZK, COVER_R * THIGH - R.get('plate_in', 0.003)),
           (-0.868 * THIGH, ZP + 0.018 * THIGH, COVER_R * THIGH - R.get('plate_in', 0.003))], DARK, 'struct', exposed=True),
    # microduck's thigh cover, its outline from its upper_leg mesh: the hull of three circles of 0.29 x the thigh's
    # length, round the hip pivot, round the knee pivot and at the top back corner (-0.868 L, +0.018 L from the hip)
    plate('L thigh cover', THIGH_Y[0], THIGH_Y[1] - THIGH_Y[0],
          [(0.0, ZP, COVER_R * THIGH), (XK, ZK, COVER_R * THIGH), (-0.868 * THIGH, ZP + 0.018 * THIGH, COVER_R * THIGH)],
          WHITE, n=96),
    # 2 screws from outside on the pivots, into the thigh plate (microduck's two marks there)
    fast(cyl('L thigh cover screw hip', (0, THIGH_Y[1] + 0.0003, ZP), 'y', 0.0019, 0.0012, DARK, 'decal'), 'M2', 0.006,
         (0, THIGH_Y[1], ZP), (0, -1, 0)),
    fast(cyl('L thigh cover screw knee', (XK, THIGH_Y[1] + 0.0003, ZK), 'y', 0.0019, 0.0012, DARK, 'decal'), 'M2', 0.006,
         (XK, THIGH_Y[1], ZK), (0, -1, 0)),
    # the plate now sits 6 mm under the cover's face: posts take those screws
    *[dict(cyl(f'L thigh post {nm}', (x_, (YT_CASE[1] + MT - 0.0005 + THIGH_Y[1] - 0.0015) / 2, z_), 'y', R.get('thigh_post_r', 0.003), THIGH_Y[1] - 0.0015 - (YT_CASE[1] + MT - 0.0005), DARK), exposed=True)
      for nm, x_, z_ in (('hip', 0.0, ZP), ('knee', XK, ZK))],
    # the wires' way from the plugs round the plate to the open inside of the cover: the windows run out through its edge
    cut(box('L thigh wire notch hip', (-(SH / 2) - 0.0005, YT_CASE[1] - 0.001, ZP + 0.0090), (-0.0105, YT_CASE[1] + MT + 0.001, ZP + 0.030), DARK), 'L thigh plate'),
    cut(box('L thigh wire notch knee', (XK - 0.030, YT_CASE[1] - 0.001, ZK + 0.0110), (XK - 0.0090, YT_CASE[1] + MT + 0.001, ZK + 0.0170), DARK), 'L thigh plate'),
    # the plate has to pass through the cover's open side as it goes on: its outline, cut all the way (a lip smaller than the plate: it could never go on. A 20 mm path check hid it)
    cut(plate('L thigh cover way', THIGH_Y[0] - 0.001, YT_CASE[1] + MT + 0.0002 - THIGH_Y[0] + 0.001, [(0.0, ZP, COVER_R * THIGH - R.get('plate_in', 0.003) + 0.0003), (XK, ZK, COVER_R * THIGH - R.get('plate_in', 0.003) + 0.0003), (-0.868 * THIGH, ZP + 0.018 * THIGH, COVER_R * THIGH - R.get('plate_in', 0.003) + 0.0003)], WHITE, 'shell'), 'L thigh cover'),
    # open on the inside, where the two servos and their horns come out
    cut(box('L thigh cover slot', (XK - SW / 2 - 0.001, YS[0] - 0.004, ZK - NEAR - 0.0015),
            (XP_FRONT + 0.0004, YT_CASE[1] - 0.0003, ZP + SW / 2 + 0.0003), WHITE, 'shell'), 'L thigh cover'),   # (its corner cut a slit through the cover's top: seen only in Fusion)
])
link('L_knee_link', 0.05, [
    box('L shin plate', (XK - 0.011 * K, YS[0], ZA + AK['box'][0] / 2 + AK['shaft_x'] - 0.001), (XK + 0.011 * K, YS[1],
                                                                                           ZK + 0.011 * K), exposed=True),
    box('L shin cap', (XK - CAP, YS[0], ZA + AK_FAR - 0.0002),
        (XK + CAP, YA_CASE[1] + (ARM if AV else 0.0), ZA + AK_FAR + 0.003), exposed=True),
    *[box('L shin arm', (XK - CAP, YA_CASE[1] - 0.0001, ZS0), (XK + CAP, YA_CASE[1] + ARM, ZA + AK_FAR + 0.0001), exposed=True) for _ in range(AV)],
    # the sleeve (2026-11-09, the user: "go down all 4 sides so the bottom servo can snugly fit into it, and we can also screw all 4 screws"): both faces and both sides, around the servo's far end, with the 2 far screws on each face
    *[box('L shin sleeve rear', (XK - CAP, YA_CASE[0] - 0.003, ZS0), (XK + CAP, YA_CASE[0] + 0.0001, ZA + AK_FAR - 0.002), exposed=True) for _ in range(AV)],
    *[fast(cyl(f'L shin arm screw {k}', (XK + dx, YA_CASE[1] + ARM + 0.00075, ZA + AK_FAR - 0.002), 'y', 0.0019, 0.0015, DARK, 'decal'), 'M2', 0.008, (XK + dx, YA_CASE[1] + ARM, ZA + AK_FAR - 0.002), (0, -1, 0)) for k, dx in enumerate((-0.008, 0.008)) if AV],
    # the shin as a real part (2026-11-03, the user: "that doesn't seem like something I can 3D print for strength"): a 3 mm
    # flat plate bent across its thin side. Below the knee's horn, 6 mm thick and 30 mm wide, a block into the corner with
    # the cap, and a collar round the servo's far end, so the load goes from the plate into the case over a box section.
    box('L shin rib', (XK - RIB, YS[0] - R.get('rib_back', 0.003), ZA + 0.0225 * K + R.get('rib_up', 0.0)), (XK + RIB, YS[1], ZK - 0.0105 * K), exposed=True),
    box('L shin gusset', (XK - RIB, YS[1] - 0.0001, ZA + AK_FAR - 0.0005), (XK + RIB, YS[1] + 0.0028, ZK - 0.0178 * K), exposed=True),
    *[box(f'L shin collar {k}', (XK + sx_ * (SW / 2 + 0.0015) - 0.0015 if not AV else XK + sx_ * (SW / 2 + 0.0003) - (0.0012 if sx_ < 0 else 0.0), YA_CASE[0] if AV else YS[1] - 0.0001, ZA + AK_FAR - 0.003 if AV else ZA + 0.028 * K),   # (the cup's sides, between the servo's faces and over its last 3 mm: lower, the shoe meets them at the ends of the ankle's range)
          (XK + sx_ * (SW / 2 + 0.0015) + 0.0015 if not AV else XK + sx_ * (SW / 2 + 0.0003) + (0.0012 if sx_ > 0 else 0.0), YA_CASE[1], ZA + AK_FAR + 0.003), exposed=True)
      for k, sx_ in enumerate((1, -1) if R.get('collars', True) or AV else ())],   # (with the new ankle, the cup's two side walls)
    # the ankle's far side, as microduck's: a plate on the servo's rear horn (4 M3 x 6), up to the shin plate, and a stub
    # from it through a small bearing in the shoe's inner ear
    *[dict(cyl('L ankle rear boss', (XK, YB + XCHH / 2, ZA), 'y', AK['horn_r'], XCHH, ALU), exposed=True) for _ in range(not HOB and not AV)],   # (a hobby servo has no rear horn: the plate lies on its rear face)
    # (the ankle servo's plugs come through this plate: PLUG. Two rails carry it past them, each side of the window)
    *[x for x in [plate('L ankle stub plate', YB - 0.002, 0.0021, [(XK, ZA, 0.0105 * K), *([(XK, ZA - NEAR + 0.0045, 0.0045)] if HOB else []), (XK - 0.0020, ZA + AK['box'][0] / 2 + AK['shaft_x'] + 0.002, 0.0103 * K), (XK + 0.0020, ZA + AK['box'][0] / 2 + AK['shaft_x'] + 0.002, 0.0103 * K)],
          DARK, 'struct', exposed=True),] if not AV],
    *[box(f'L ankle rail {k}', (XK + sx_ * 0.01095 - 0.0011, YB - 0.0034, ZA + 0.0090), (XK + sx_ * 0.01095 + 0.0011, YB + 0.0001, ZA + 0.0250), DARK, exposed=True)
      for k, sx_ in enumerate((1, -1)) if LEG_STS and not AV],
    *[x for x in [dict(cyl('L ankle stub', (XK, YB - 0.0041, ZA), 'y', ANK['d'] / 2 - 0.00005, 0.0042, DARK), exposed=True),] if not AV],   # (0.1 mm under the bearing's bore: FDM prints pegs large)
    # flat heads, flush: the bearing's race turns 0.2 mm off this plate
    # (it was M3 at 45 degrees on every variant: an XL330's idler has M2 holes at 0 and 90)
    *[fast(cyl(f'L ankle boss screw {k}', (XK + AK['horn_hole_r'] * math.cos(math.radians(AK['horn_hole_deg'][k])), YB - 0.00175,
                                           ZA + AK['horn_hole_r'] * math.sin(math.radians(AK['horn_hole_deg'][k]))), 'y', 0.0027 if 'steps' in AK else 0.0019, 0.0005, DARK, 'decal'),
           'M3 flat' if 'steps' in AK else 'M2 flat', 0.006 if 'steps' in AK else 0.004 if 'centre_screw' in AK else 0.0048 if R.get('horn_recess') else 0.005, (XK + AK['horn_hole_r'] * math.cos(math.radians(AK['horn_hole_deg'][k])), YB - 0.002,
                              ZA + AK['horn_hole_r'] * math.sin(math.radians(AK['horn_hole_deg'][k]))), (0, 1, 0), **({'clear': 0.0012, 'recess': (0.0021, 0.0008)} if R.get('horn_recess') else {})) for k in range(0 if HOB or AV else 4)],
    *[x for x in [dict(cyl('L ankle bearing inner race', (XK, YB - 0.0042, ZA), 'y', ANK['d'] / 2 + RACE, ANK['w'], ALU), exposed=True,
         material='steel', mass=ANK['mass'] / 2, on_joint='L_ankle_pitch'),
    cut(cyl('L ankle bearing inner race bore', (XK, YB - 0.0042, ZA), 'y', ANK['d'] / 2 - 0.00005, ANK['w'] + 0.002),
        'L ankle bearing inner race'),] if not AV],
])
SOLE_C = ((FOOT['x'][0] + FOOT['x'][1]) / 2, (FOOT['y'][0] + FOOT['y'][1]) / 2)
SOLE_A, SOLE_B = (FOOT['x'][1] - FOOT['x'][0]) / 2 + 0.0005, (FOOT['y'][1] - FOOT['y'][0]) / 2 + 0.0005
EAR = [(XK, ZA, 0.011 * K), (XK - 0.013, ZF + 0.0031, 0.003), (XK + 0.013, ZF + 0.0031, 0.003)] if AV else [(XK, ZA, 0.011 * K), (XK - 0.0175 * K, FOOT['rim'] - 0.006, 0.005), (XK + 0.021 * K, FOOT['rim'] - 0.006, 0.005)]   # inside the shoe's round corners
link('L_foot', 0.05, [
    # the shoe's ears (microduck's): orange, rising either side of the ankle servo to round tops on its axis; the
    # outer one on the horn, the inner one on a bearing round the shin's stub (the shoe joins them: no post). The inner
    # ear is its own part: a one-piece shoe cannot go on, its two ears fixed either side of a servo with a stub (found
    # tracing the assembly order). The shoe rises onto the servo, then this ear slides onto the stub, its two lobes in a
    # recess in the shoe, 2 screws into the shoe.
    plate('L ankle ear out', YA - 0.0005, 0.0035, EAR, ORANGE, 'struct', exposed=True),
    # (3 mm thick, 4 round the bearing: its lobes swing past the ankle servo's plugs at the far ends of the range)
    plate('L ankle ear in', YB - EAR_GAP - EAR_T if AV else YB - 0.0067, EAR_T if AV else 0.003, EAR, ORANGE, 'struct', exposed=True),
    *[x for x in [dict(cyl('L ankle ear hub', (XK, YB - 0.0047, ZA), 'y', 0.011 * K, 0.004, ORANGE), exposed=True),
    cut(cyl('L ankle ear bore', (XK, YB - 0.0047, ZA), 'y', ANK['D'] / 2 - 0.00005, 0.006, ORANGE), 'L ankle ear in', 'L ankle ear hub'),
    cut(plate('L ankle ear recess', FOOT['y'][0] - 0.001, YB - 0.0026 - FOOT['y'][0] + 0.001, EAR, ORANGE, 'struct'), 'L shoe'),
    *[fast(cyl(f'L ankle ear screw {k}', (XK + dx, YB - 0.0073, FOOT['rim'] - 0.006), 'y', 0.0019, 0.0012, DARK, 'decal'),
           'M2', 0.008, (XK + dx, YB - 0.0067, FOOT['rim'] - 0.006), (0, 1, 0)) for k, dx in enumerate((-0.0175 * K, 0.021 * K))],
    dict(cyl('L ankle bearing outer race', (XK, YB - 0.0042, ZA), 'y', ANK['D'] / 2, ANK['w'], ALU), exposed=True,
         material='steel', mass=ANK['mass'] / 2, on_joint='L_ankle_pitch'),
    cut(cyl('L ankle bearing outer race bore', (XK, YB - 0.0042, ZA), 'y', ANK['D'] / 2 - RACE, ANK['w'] + 0.002),
        'L ankle bearing outer race'),] if not AV],
    *([dict(cyl('L ankle rear boss', (XK, YB + (XCHH - EAR_GAP) / 2, ZA), 'y', AK['horn_r'], XCHH + EAR_GAP, ALU), exposed=True),   # (drawn up to the ear, the gap closed as the screws pull it down)
       cut(cyl('L ankle ear access', (XK, YB - EAR_GAP - EAR_T / 2, ZA), 'y', 0.0032, EAR_T + 0.002, ORANGE), 'L ankle ear in'),
       *[fast(cyl(f'L ankle ear screw {k}', (XK + 0.006 * math.cos(math.radians(d)), YB - EAR_GAP - EAR_T - 0.00075, ZA + 0.006 * math.sin(math.radians(d))), 'y', 0.0019, 0.0015, DARK, 'decal'), 'M2', 0.006,
              (XK + 0.006 * math.cos(math.radians(d)), YB - EAR_GAP - EAR_T, ZA + 0.006 * math.sin(math.radians(d))), (0, 1, 0), clear=0.0012) for k, d in enumerate((0, 90, 180, 270))]] if AV else []),
    # microduck's shoe is one rounded block (round 11, the user's side-by-side): the orange upper on the yellow sole,
    # big round edges, the ears rising out of it; the ankle servo sits down in a pocket between the ears
    # (the upper has the sole's outline exactly, so the two read as one block split by colour; it rounds over on top)
    dict(loft('L shoe', [*([(ZF, SOLE_C[0], SOLE_C[1], SOLE_A, SOLE_B, 4.5)] if AV else [(FOOT['sole'] - 0.001, SOLE_C[0], SOLE_C[1], SOLE_A - 0.0004, SOLE_B - 0.0004, 4.5),
                         (FOOT['sole'], SOLE_C[0], SOLE_C[1], SOLE_A, SOLE_B, 4.5)]),
                         (FOOT['rim'] - 0.004, SOLE_C[0], SOLE_C[1], SOLE_A, SOLE_B, 4.5),
                         (FOOT['rim'] - 0.0012, SOLE_C[0], SOLE_C[1], SOLE_A - 0.0025, SOLE_B - 0.0025, 4.0),
                         (FOOT['rim'], SOLE_C[0], SOLE_C[1], SOLE_A - 0.0045, SOLE_B - 0.0045, 3.5)], ORANGE, 'struct',
              open=()), exposed=True),
    # (the horn stands 1.55 mm off the case's face, past the cup, and its lower edge is under the shoe's rim: room for it)
    *[cut(cyl('L shoe horn relief', (XK, YA_CASE[1] + ACT[R['leg']]['horn_top'] / 2 + 0.0001, ZA), 'y', ACT[R['leg']]['horn_r'] + 0.0004, ACT[R['leg']]['horn_top'] + 0.0002, ORANGE, 'shell'), 'L shoe') for _ in range(bool(R.get('shoe_horn_relief')))],
    # a round cup for the servo's corners (16 mm from the shaft), down into the TPU sole, as microduck's ankle cup
    cut(cyl('L shoe pocket', (XK, ((YB if AV else YB - 0.0025) + YA_CASE[1] + (ARM + 0.0003 if AV else 0.0)) / 2, ZA), 'y', 0.021 * K, YA_CASE[1] + (ARM + 0.0003 if AV else 0.0) - (YB if AV else YB - 0.0025),
            ORANGE, 'shell'), *(('L shoe',) if AV else ('L shoe', 'L sole'))),
    # (a small duck's cup only just dips through the shoe's floor, and leaves it feather edges a printer can't make: cut them square)
    # (the shoe's wall stood in the horn's disc, 7-10 mm below the shaft, the 1.55 mm of the horn's whole height: 45 mm3 measured on Fusion's body, so the horn could never seat on its plate. Room for the disc, from the case's face to the horn's top)
    *[cut(cyl('L shoe horn relief', (XK, YA - 0.002925, ZA), 'y', 0.0104, 0.00215, ORANGE), 'L shoe') for _ in range(bool(R.get('shoe_horn_relief')))],
    *[cut(box('L shoe pocket floor', (XK - 0.0085, YB if AV else YB - 0.0025, FOOT['sole'] - 0.002), (XK + 0.0085, YA_CASE[1], FOOT['sole'] + 0.0006), ORANGE, 'shell'), 'L shoe') for _ in range(not LEG_STS)],
    # two pieces, as microduck's shoe (its foot and sole meshes) and Open Duck's (a TPU bottom, screwed on from below). The
    # foot switch (2026-11-02, the user: an input to the walking policy, as Open Duck's GPIO foot switches): a subminiature
    # microswitch pressed into the shoe's underside ahead of the ankle pocket, its plunger on the TPU sole, which gives
    # as the foot takes weight. Its travel against this TPU is untested: try it before printing all four
    # (the all-Dynamixel duck has none, 2026-11-05, the user: "too much wire", and "just servo states would be good enough": microduck has no switch either)
    *([cut(box('L foot switch pocket', (XK + 0.0285 * K - 0.0065, SOLE_C[1] - 0.0030, FOOT['sole'] - 0.0015 - R.get('switch_drop', 0)),
            (XK + 0.0285 * K + 0.0065, SOLE_C[1] + 0.0030, FOOT['sole'] + 0.0052 - R.get('switch_drop', 0)), ORANGE, 'struct'), 'L shoe', 'L sole'),
       box('L foot switch', (XK + 0.0285 * K - 0.0064, SOLE_C[1] - 0.0029, FOOT['sole'] - 0.0015 - R.get('switch_drop', 0)),
        (XK + 0.0285 * K + 0.0064, SOLE_C[1] + 0.0029, FOOT['sole'] + 0.005 - R.get('switch_drop', 0)), DARK, material='electronics', mass=0.001, exposed=True)] if R.get('foot_switch', True) else []),
    *[fast(cyl(f'L shoe bottom screw {k}', (XK + dx, SOLE_C[1], 0.0078 * K), 'z', 0.0027, 0.0015, DARK, 'decal'), 'M3', 0.012,
           (XK + dx, SOLE_C[1], 0.008 * K), (0, 0, 1)) for k, dx in enumerate((-0.0215 * K, 0.038 * K))],
    # the sole curls up all round underneath, toe and heel most (microduck's, round 11): sections from a smaller
    # rounder footprint on the floor out to the full one 5 mm up; it stands on the flat middle ('contact')
    dict(loft('L sole', [(0.0, SOLE_C[0], SOLE_C[1], SOLE_A - 0.009, SOLE_B - 0.005, 2.6),
                         (0.0015, SOLE_C[0], SOLE_C[1], SOLE_A - 0.004, SOLE_B - 0.0022, 3.2),
                         (0.0040, SOLE_C[0], SOLE_C[1], SOLE_A - 0.0008, SOLE_B - 0.0004, 4.0),
                         (0.0065, SOLE_C[0], SOLE_C[1], SOLE_A, SOLE_B, 4.5),
                         (ZF if AV else FOOT['sole'], SOLE_C[0], SOLE_C[1], SOLE_A, SOLE_B, 4.5)], YELLOW, 'struct', open=()),
         exposed=True, contact=dict(c=[SOLE_C[0], SOLE_C[1], 0.002], size=[2 * SOLE_A - 0.020, 2 * SOLE_B - 0.010, 0.004])),
])

# ranges in microduck's stance (the zero pose): the hip folds forward 1.3 and back 0.43, the knee 1.1 each way
# the driven horn and a bearing, as microduck's (BEARINGS); no rear horns, forks or posts
joint('L_hip_yaw', 'trunk', 'L_hip_yaw_link', (0, YY, ZY), 'z', (-0.45, 0.45), R['yaw'], out=-1, long='-x',
      fork=False, bearing=('horn',), near=True)
joint('L_hip_roll', 'L_hip_yaw_link', 'L_hip_roll_link', (XRH, YR, ZR), 'x', (-0.33, 0.33), R['leg'], out=+1,
      long='-z', **(dict(fork=True, post=False, bearing=(), sides=(), faces=(), tray_full=False, near=False, rear_stack=0.0035) if ROLLF else dict(fork=False, bearing=('horn',), sides=(1,), tray_full=True, near=True)))   # no inner cradle wall (the two roll cradles face each other)
joint('L_hip_pitch', 'L_hip_roll_link', 'L_hip_pitch_link', (0, YP, ZP), 'y', RANGE['hip'], R['leg'], out=-1,
      long='-x', case_in='child', fork=False, bearing=('horn',), near=True)
joint('L_knee', 'L_hip_pitch_link', 'L_knee_link', (XK, YK, ZK), 'y', RANGE['knee'], R['leg'], out=-1,
      long='+z', fork=False, bearing=('horn',), near=True)
# the bearing on the far side, in the shoe's inner ear (spelled out in the links); no side walls: they dip into the shoe
joint('L_ankle_pitch', 'L_knee_link', 'L_foot', (XK, YA, ZA), 'y', RANGE['ankle'], R['leg'], out=+1, long='+z',
      fork=False, sides=(), stack=ANKLE_STACK, faces=('rear',))   # (no lid: a 0.2 cm3 strip, the rear plate and idler hold this side. The user: why two parts?)

# ---------------------------------------------------------------- neck and head (bare servos, as microduck's)

# microduck's head (round 5, 2026-11-02; the user: "check the inner groove vs ours flat... match the sharp straight
# front with rounded-to-back curves... did you even add a grid?"). Measured on microduck's top_head_shell, face_part,
# noenoeil and jaw meshes in 5 mm steps (tools/compare.py --head prints the grid; the meshes stay outside the repo),
# then scaled by HK, milli's height over microduck's. The hood is a loft of front-view arches stacked from back to
# front (a superellipse of power 3 standing on the hood's lower edge): its top rises gently to a sharp, straight,
# upright front; the hood is widest at the front and narrows and rounds down toward the back. The face is a grey
# arch set 20 mm into the hood's open front, the same arch shrunk by the rim; one big amber eye ring in its middle.
HK = R['HK']                                          # the head at the scale the STS3215 body sets (milli / microduck)
# (distance back from the front, height above the hood's lower edge, half width) on microduck, metres
# (the heights from the front back to 92 mm smoothed to the straight slope they scatter about: raw, they left a dip)
HOOD_MD = [(0.1227, 0.0141, 0.0229), (0.1173, 0.0241, 0.0280), (0.1123, 0.0295, 0.0331), (0.1023, 0.0357, 0.0364),
           (0.0923, 0.0381, 0.0388), (0.0623, 0.0408, 0.0407), (0.0323, 0.0435, 0.0440), (0.0123, 0.0453, 0.0459),
           (0.0, 0.0464, 0.0458)]
HEAD_FRONT = HEAD_CX + 0.085 * HK / 1.18
HEAD_BACK = HEAD_FRONT - HK * HOOD_MD[0][0]
RIM = 0.0025                                       # the hood's rim round the face


def hood(grow=0.0, n_front=3.0, n_back=2.4):
    """The hood's sections along x, back to front: (x, lower edge, 0, height, half width, power)."""
    z0 = HEAD_Z + BILL_H                           # the hood stands on the upper bill (round 5: it overlapped it)
    tail_ = R.get('hood_tail', (0.004, 0.006, 0.014))   # the back cap: how far behind the first section, its height, its half width
    out = [(HEAD_BACK - tail_[0], z0, 0, tail_[1] + grow, tail_[2] + grow, n_back)]
    for k, (d, h, w) in enumerate(HOOD_MD):
        n = n_back + (n_front - n_back) * k / (len(HOOD_MD) - 1)
        out.append((HEAD_FRONT - HK * d, z0, 0, HK * h + grow, HK * w + grow, n))
    out.append((HEAD_FRONT + 0.012,) + out[-1][1:])   # past the front: the leaning front plane cuts it
    return out


def arch_front(x0, x1, shrink, z_low=None):
    """The front arch (the hood's front section shrunk by `shrink`) as a slab from x0 to x1."""
    _, cz, _, a, b, n = hood()[-1]
    return [(x0, cz, 0, a - shrink, b - shrink, n), (x1, cz, 0, a - shrink, b - shrink, n)]


BILL_H = 0.005                                     # the upper bill: an orange band round the hood's lower edge
UNDER = box('head underside', (HEAD_BACK - 0.01, -0.08, HEAD_Z - 0.09), (HEAD_FRONT + 0.03, 0.08, HEAD_Z), WHITE,
            'shell')
ABOVE_BILL = box('above the bill', (HEAD_BACK - 0.01, -0.08, HEAD_Z + BILL_H), (HEAD_FRONT + 0.03, 0.08,
                                                                                   HEAD_Z + 0.09), WHITE, 'shell')
FACE_X = HEAD_FRONT - HK * 0.011                   # the face's plane (microduck's face_part: 11 mm in at its edges)
# the hood's front leans forward 6 deg toward the top, over the face (microduck's, on the grid)
FRONT_LEAN = 0.105
FRONT = ((HEAD_FRONT, 0, HEAD_Z), (-1.0, 0, FRONT_LEAN))
EYE = dict(y=0.0, z=HEAD_Z + HK * 0.020, r=HK * 0.015)
HEAD_PLATE_Z = HEAD_Z + R.get('head_plate', 0.029 * Q)                      # the head's top plate, on the beak servo, under the hood's back

RAIL = R.get('rail', 0.034)                        # the head frame's half width: outside the roll case's reach as the head rolls
BOSS_IN = R.get('boss_in', 0.003)                  # the hinge boss's outer face, in from the hood: 1 mm clear of its 2 mm wall. Closer, its corners came through where the hood curves in
YAW_R = math.hypot(XLL / 2 + XLX, XLW / 2) + GAP   # what turns under the head with its yaw: the yaw case's reach
link('neck_link', 0.02, [
    box('neck front plate', (XN + NKW / 2 - 0.0005, -0.0098 * Q, ZN + 0.008 * Q), (XN + NKW / 2 + 0.0025, R.get('neck_plate_top', 0.010 * Q), ZH - 0.0145 * Q),   # (it stops under the head pitch servo's wires)
        exposed=True),
    box('neck back plate', (XN - NKW / 2 - 0.0025, -0.0098 * Q, ZN + 0.008 * Q), (XN - NKW / 2 + 0.0005, R.get('neck_plate_top', 0.010 * Q), ZH - 0.0145 * Q),
        exposed=True),
])
link('head_pitch_link', 0.03, [
    box('head pitch plate', (XN - 0.010 * Q, -0.019 * Q, ZH - 0.011 * Q), (XN + 0.010 * Q, -0.0155 * Q, YAW_FOOT), exposed=True),
    box('head pitch idler plate', (XN - 0.010 * Q, PITCH_IDLER[0], ZH - 0.011 * Q), (XN + 0.010 * Q, PITCH_IDLER[1], YAW_FOOT), exposed=True),
    box('head pitch bridge', (min(XN - NKL / 2, XHY - XLX - XLL / 2), -0.019 * Q, YAW_FOOT - 0.003), (max(XN + NKL / 2, XHY - XLX + XLL / 2), max(0.020 * Q, PITCH_IDLER[1]), YAW_FOOT),
        exposed=True),
])
link('head_yaw_link', 0.02, [
    # (the bearing's shoulder stops at the trimmed plate's edge: past it, a 0.6 mm flap)
    *[cut(box('head yaw shoulder chord', (XHY - 0.02, R['yaw_plate_y'], ZHY - 0.003), (XHY + 0.02, 0.02, ZHY + 0.001), DARK), 'head yaw horn boss shoulder') for _ in range('yaw_plate_y' in R)],
    box('head yaw plate', (XHY - XLH / 2, -0.0255 * Q, ZHY - R.get('yaw_plate_low', 0.0005)), (XHY + XLH / 2, R.get('yaw_plate_y', 0.009 * Q), ZHY + XLHH + YAW_UP), exposed=True),
])
# the beak servo: inside the head over the neck, its horn on the jaw's hinge at mid-head, as microduck's side tab
# the beak servo lies flat behind the head yaw and roll servos, its top under the top plate (round 9: the head at
# milli's smaller scale is too low at the back for it standing)
BEAK_STACK = R.get('beak_stack', XLHH)              # the beak servo's face to the jaw tab
XBK, ZBK = XN - R.get('beak_x', 0.039 * Q) - HB, HEAD_Z + R.get('beak_z', 0.019 * Q)              # over the head yaw case; behind the head roll's rear bearing
def interp(x, xs, ys):
    """Linear interpolation, ends held."""
    if x <= xs[0]:
        return ys[0]
    for k in range(1, len(xs)):
        if x <= xs[k]:
            return ys[k - 1] + (ys[k] - ys[k - 1]) * (x - xs[k - 1]) / (xs[k] - xs[k - 1])
    return ys[-1]


JAW_Z = (HEAD_Z - 0.0115, HEAD_Z - 0.0010)       # with the upper bill, microduck's 16.5 mm band
# the jaw's hinge (round 6): on the side tabs at mid-head, over the neck, as microduck's; the beak servo behind the
# head yaw servo turns it through a rod (microduck's jaw is driven through a linkage too)
XJ, ZJ = XHY + R.get('xj_ahead', 0.003), ZBK                          # (3 mm ahead of the yaw axis: the beak servo's far corner came through the hood where it curves in)
JAW_X0 = XJ - 0.002                                # the jaw's sides start at the hinge: behind it they rise into the bill


def band_secs(x0, x1):
    """The hood's outline (grown 0.6 mm, as the upper bill) as a 12.5 mm band under the hood, from x0 to x1."""
    out = [(x, sum(JAW_Z) / 2, 0, (JAW_Z[1] - JAW_Z[0]) / 2, b + 0.0006, 8.0) for x, _, _, a, b, n in hood()]
    xs = [o[0] for o in out]
    bx = lambda x: interp(x, xs, [o[4] for o in out])
    mid = [o for o in out if x0 < o[0] < x1]
    return [(x0, out[0][1], 0, out[0][3], bx(x0), 8.0)] + mid + [(x1, out[0][1], 0, out[0][3], bx(x1), 8.0)]


def hood_b(x):
    h = hood()
    return interp(x, [o[0] for o in h], [o[4] for o in h])


# the hinge tab, in a step in the hood's side, under a lip (2026-11-04, the user: "it has a lip under which the jaw can slide": it stood 5 mm off a full-width hood, and poked out). Microduck's shell
# narrows upward and is cut away behind its tab. Ours steps in instead of being cut through, as the hinge screws hold the hood on: 1.3 mm proud of the hood
TAB_Y_ = max(hood_b(XJ - 0.014), hood_b(XJ + 0.014)) - 0.004     # 4 mm thick, its outer face flush with the hood; the screw heads sink 1 mm. (The servo sits that far in, clear of the hood's curve)
BOSS_IN = hood_b(XJ) - (TAB_Y_ - 0.0026)           # the boss's outer face, under the step's floor


def jaw_recess():
    """Each side: a block inside the hood, its outer face the floor of the step, and the step cut through the hood's wall and band. The outline: the tab, as far as the jaw turns, 1 mm clear."""
    tab = [(XJ, ZJ + 0.002, 0.011), (JAW_X0 + 0.0015, JAW_Z[0] + 0.0015, 0.0015), (XJ + 0.018, JAW_Z[0] + 0.004, 0.004)]
    turn = lambda q, c: [(XJ + (x - XJ) * math.cos(q) - (z - ZJ) * math.sin(q), ZJ + (x - XJ) * math.sin(q) + (z - ZJ) * math.cos(q), r + c) for x, z, r in tab]
    # only as far as it really goes, closed to fully open, front going down: a slot round the tab, nothing more (the user: "no further extra space, for a sleek look"; swept both ways it was a big triangle)
    swept = lambda c: [p for q in (0.0, -R.get('beak_max', 0.4) / 2, -R.get('beak_max', 0.4) - 0.02) for p in turn(q, c)]
    floor = TAB_Y_ - 0.0005
    out = []
    for sy, nm in ((1, 'L'), (-1, 'R')):
        hb = hood_b(XJ) - 0.0005
        if sy > 0:                                  # on the horn side the wall is simply cut away behind the tab, as microduck's: the servo's horn comes through it
            out.append(cut(plate(f'{nm} hood step cut', TAB_Y_ - 0.0035, 0.03, swept(0.0005), WHITE, 'struct'), 'head', 'upper bill'))
            continue
        out.append(dict(plate(f'{nm} hood step', floor - 0.002 if sy > 0 else -hb, hb - floor + 0.002, swept(0.002), WHITE, 'decal'), exposed=True))   # (part of the hood, a shell, not of the head's frame)
        out.append(cut(plate(f'{nm} hood step cut', floor if sy > 0 else -floor - 0.03, 0.03, swept(0.0005), WHITE, 'struct'), 'head', 'upper bill', f'{nm} hood step'))
        out.append(cut(box(f'{nm} hood step under', (XJ - 0.06, -0.08, HEAD_Z - 0.06), (XJ + 0.06, -0.01, HEAD_Z + 0.0002), WHITE), f'{nm} hood step'))
    return out


# The mouth, as microduck's (2026-11-03, the user: "our mouth is hollow square"; its launch video and its jaw, jaw_soft and
# soft_mouth_top meshes): the jaw is a tray from the lip back 27 mm (x 1.18), then two thin arms to the hinge. A soft insert
# fills the tray, 1 mm under its rim, and another closes the roof. Printed in TPU, glued in.
SOFT = '#F7AE55'
MOUTH_X = HEAD_FRONT - HK * 0.027                # the tray's back edge
MOUTH_F = HEAD_FRONT - 0.0025


def mouth_outline(x0):
    """The insert's outline from x0 to the lip, 1.6 mm inside the band (axis z: circles (x, y, r))."""
    r = 0.004
    return [(x_ + (r if x_ == x0 else -r), sy * (hood_b(x_) - 0.0016 - r), r) for x_ in (x0, MOUTH_F) for sy in (1, -1)]


HRX_ = HO((R.get('hood_rear_x', -0.0192), 0, 0))[0]               # the hood's 2 rear screws, in the band's solid back
HRX2_ = (HRX_ - 0.0005, HRX_ + 0.0005)
PI_X, PI_Z, PI_T = FACE_X - R.get('pi_back', 0.0268), HEAD_Z + 0.021 * Q, 0.0052    # the Pi: its plate's front face, its board's middle height, its thickness with parts
BUS_Z = HEAD_PLATE_Z + 0.0092                      # under the bus board
# the beak's linkage, at the jaw's shut pose (x, z)
BEAK_Y = R.get('beak_y', (0.033, 0.036))           # the rod's plane
BEAK_CRANK = R.get('beak_crank', (0.014, 0.55))    # its radius, its lean forward when shut (rad)
BEAK_PIN = (XBK + BEAK_CRANK[0] * math.sin(BEAK_CRANK[1]), ZBK - BEAK_CRANK[0] * math.cos(BEAK_CRANK[1]))
JAW_PIN = (XJ, ZJ - R.get('jaw_drop', 0.0199))
# the all-Dynamixel duck's bus board, the one on the user's Koch arm: Waveshare Bus Servo Adapter (A), v1.1. Size, holes and connectors from Waveshare's dimension drawing (docs.waveshare.com/Bus_Servo_Adapter_A):
# 42 x 33 mm, 4 x 2.5 mm holes at 37 x 28; on the 37's edge at its top, 2 sockets, opening away from the board, and the UART pins; on the other, the DC jack, screw terminals and USB-C. Parts' heights read off
# photographs, the tallest (the jack) taken as 11 mm. It stands on the back of the Pi's plate, its parts toward the head frame, its 42 upright. No flat place has the room (the roll servo's sweep reaches 3 mm under the hood).
# Its 4 brass hex standoffs: their 5 mm across flats and 8 mm length read off a photograph: measure yours. They sit in hex sockets, so a screw at each end can be turned against them.
BB = dict(L=0.042, W=0.033, hx=0.037, hy=0.028, t=0.0016, parts=0.011, so=0.008, af=0.005)
BB_Z = PI_Z + 0.0043                               # its middle height
BB_FLOOR = PI_X - 0.0014                           # a socket's floor, 0.6 mm into the Pi plate


def bus_board():
    pcb = BB_FLOOR - BB['so']                      # the board's back
    top = pcb - BB['t']                            # its parts' side
    z_ = lambda bx: BB_Z + BB['L'] / 2 - bx        # along the drawing's 42: the jack's end up, clear of the head frame's screws
    y_ = lambda by: BB['W'] / 2 - by               # across it: 0 at the jack's edge, on the +y side
    holes = [(sy * BB['hy'] / 2, BB_Z + sz * BB['hx'] / 2) for sy, sz in ((1, 1), (-1, 1), (1, -1), (-1, -1))]
    hexa = lambda c, af: [(c[0] + af / 3 ** 0.5 * math.cos(math.radians(60 * k)), c[1] + af / 3 ** 0.5 * math.sin(math.radians(60 * k)), 1e-5) for k in range(6)]
    out = [box('Pi board tab', (PI_X - 0.002, -BB['W'] / 2 - 0.001, BB_Z - BB['L'] / 2 - 0.0005), (PI_X, BB['W'] / 2 + 0.001, BB_Z + BB['L'] / 2 + 0.0005))]
    for k, c in enumerate(holes):
        out += [dict(cyl(f'bus board post {k}', (PI_X - 0.002 - 0.0015, c[0], c[1]), 'x', 0.0042, 0.003, DARK), exposed=True),
                cut(plate(f'bus board socket {k}', BB_FLOOR - 0.0055, 0.0055, hexa(c, BB['af'] + 0.0002), DARK, 'struct', axis='x'), f'bus board post {k}', 'Pi board tab', 'Pi plate', 'Pi base tab', 'head base'),
                dict(plate(f'bus board standoff {k}', BB_FLOOR - BB['so'], BB['so'], hexa(c, BB['af']), '#C9A24A', 'struct', axis='x'), material='electronics', mass=0.0006),
                fast(cyl(f'bus board top screw {k}', (top - 0.00075, c[0], c[1]), 'x', 0.00225, 0.0015, DARK, 'decal'), 'M2.5', 0.005, (top, c[0], c[1]), (1, 0, 0)),
                fast(cyl(f'bus board rear screw {k}', (PI_X + 0.00075, c[0], c[1]), 'x', 0.00225, 0.0015, DARK, 'decal'), 'M2.5', 0.005, (PI_X, c[0], c[1]), (-1, 0, 0))]
    # its real shape (a box round all of it stood on the head roll's screw heads): the board, and each part on it: (along the 42, across the 33 from the jack's edge, height)
    pp_ = [((4.9, 14.5), (-1.1, 14), 11.0), ((17.3, 25.3), (-1, 6.3), 10.0), ((28, 35.1), (-1.3, 7.4), 3.3), ((5.2, 15.9), (28, 33), 6.0), ((17.5, 28.3), (28, 33), 6.0), ((29.6, 37.0), (30.5, 33), 8.5), ((33.6, 39.3), (14.2, 22), 6.0)]
    bx_ = [[[top, -BB['W'] / 2, BB_Z - BB['L'] / 2], [pcb, BB['W'] / 2, BB_Z + BB['L'] / 2]]]
    for (u0, u1), (v0, v1), h in pp_:
        bx_.append([[top - h / 1000, y_(v1 / 1000), z_(u1 / 1000)], [top + 0.0001, y_(v0 / 1000), z_(u0 / 1000)]])
    out.append(dict(box('Waveshare bus board', (top - BB['parts'], -BB['W'] / 2, BB_Z - BB['L'] / 2), (pcb, BB['W'] / 2, BB_Z + BB['L'] / 2), PCB, material='electronics', mass=0.012), kind='stepbox', boxes=bx_))
    # what plugs into it needs room past its edges: the servo leads out of the sockets, the power leads into the terminals, a right-angle USB-C lead
    wired = lambda n, x0, ys, bxs: dict(box(f'bus board cable plugs {n}', (x0, min(ys), z_(bxs[1])), (top, max(ys), z_(bxs[0])), '#D9D4C5', fillet=0.0003), material='electronics', mass=0.0005, wire=True, exposed=True)
    out += [wired('servo', top - 0.008, (y_(BB['W']), y_(BB['W']) - 0.007), (5.2e-3, 28.3e-3)),
            wired('power', top - 0.010, (y_(0), y_(0) + 0.007), (17.3e-3, 25.3e-3)),
            *([wired('usb', top - 0.0045, (y_(0), y_(0) + 0.0115), (28.0e-3, 35.1e-3))] if not R.get('pi_uart') else
              # (pi_uart: the UART header's 3 pins snipped to 3 mm over their spacer, the wires soldered on and laid flat, out past the board's edge)
              [wired('uart', top - 0.0055, (y_(BB['W']), y_(BB['W']) - 0.004), (29.6e-3, 37.0e-3))])]
    return out


def rear_board():
    """The Feetech bus's Waveshare board, the same board as bus_board()'s, on a plate behind the head roll servo: the room at the back of the head, its weight behind the head pitch axis, where the head wants it, and its leads a short way from the neck's opening. Flat on 4 printed bosses, its parts toward the hood. Its 42 across, jack and terminals down, where the hood reaches furthest back, sockets up."""
    X0, X1 = 0.0090, 0.0116                        # the plate: its front on the beak cradle's end and the riser's top
    pcb = X0 - 0.003
    top = pcb - BB['t']
    Z0 = HEAD_Z + 0.0075                           # its low edge: the plate clears the head yaw tray as the head rolls
    y_ = lambda u: BB['L'] / 2 - u
    z_ = lambda v: Z0 + v
    ZC = Z0 + BB['W'] / 2
    # (its middle stops 7 mm up: as the head rolls, that low edge comes down to the head yaw tray)
    out = [box('rear board plate', (X0, -0.024, Z0 + 0.007), (X1, 0.024, ZC + BB['W'] / 2 + 0.0005)),
           box('rear board plate low', (X0, 0.015, Z0 - 0.0005), (X1, 0.024, Z0 + 0.0075)), box('rear board plate low 2', (X0, -0.024, Z0 - 0.0005), (X1, -0.015, Z0 + 0.0075)),
           box('rear board foot', (X0, 0.0235, HEAD_Z + 0.0045), (X1, 0.0310, Z0 + 0.004)), box('rear board foot 2', (X0, -0.0310, HEAD_Z + 0.0045), (X1, -0.0235, Z0 + 0.004))]    # (2 feet, down onto the base's edge, either side of its opening: a full-width strip met the head yaw tray as the head rolled)
    for k, (sy, sz) in enumerate(((1, 1), (-1, 1), (1, -1), (-1, -1))):
        c = (sy * BB['hx'] / 2, ZC + sz * BB['hy'] / 2)
        out += [dict(cyl(f'rear board boss {k}', (pcb + 0.0015, c[0], c[1]), 'x', 0.0030, 0.003, DARK), exposed=True),
                fast(cyl(f'leg bus board screw {k}', (top - 0.00075, c[0], c[1]), 'x', 0.00225, 0.0015, DARK, 'decal'), 'M2.5', 0.005, (top, c[0], c[1]), (1, 0, 0))]
    pp_ = [((4.9, 14.5), (-1.1, 14), 11.0), ((17.3, 25.3), (-1, 6.3), 10.0), ((28, 35.1), (-1.3, 7.4), 3.3), ((5.2, 15.9), (28, 33), 6.0), ((17.5, 28.3), (28, 33), 6.0), ((29.6, 37.0), (30.5, 33), 8.5), ((33.6, 39.3), (14.2, 22), 6.0)]
    bx_ = [[[top, -BB['L'] / 2, Z0], [pcb, BB['L'] / 2, Z0 + BB['W']]]]
    for (u0, u1), (v0, v1), h in pp_:
        bx_.append([[top - h / 1000, y_(u1 / 1000), z_(v0 / 1000)], [top + 0.0001, y_(u0 / 1000), z_(v1 / 1000)]])
    out.append(dict(box('leg bus board', (top - BB['parts'], -BB['L'] / 2, Z0 - 0.0013), (pcb, BB['L'] / 2, Z0 + BB['W']), PCB, material='electronics', mass=0.012), kind='stepbox', boxes=bx_))
    wired = lambda n, x0, us, zs: dict(box(f'leg bus board plugs {n}', (x0, y_(us[1]), zs[0]), (top, y_(us[0]), zs[1]), '#D9D4C5', fillet=0.0003), material='electronics', mass=0.0005, wire=True, exposed=True)
    # the neck lead, plugged straight into socket 1; 3 wires on the UART pins; the 2S pair and the regulator's pair into the terminals, from below
    out += [wired('servo', top - 0.009, (5.2e-3, 15.9e-3), (z_(0.028), z_(0.033))),
            wired('uart', top - 0.0115, (29.6e-3, 37.0e-3), (z_(0.0305), z_(0.033))),
            dict(box('leg bus board plugs power', (top - 0.008, y_(25.3e-3), Z0 - 0.0065), (top - 0.002, y_(17.3e-3), Z0 - 0.0009), '#D9D4C5', fillet=0.0003), material='electronics', mass=0.0005, wire=True, exposed=True)]
    return out


link('head', 0.12, [
    # the head's frame: a plate in front of the head roll servo, a plate on the beak servo behind it, two rails
    # past it either side (outside the roll servo's 27 mm reach as the head rolls)
    box('head frame', (XHR - R.get('frame_back', 0.0005), -RAIL, ZHY + 0.004 * Q), (XHR + 0.003, RAIL, HEAD_PLATE_Z + 0.003)),
    # narrow at the hood's low back, wide beside the roll servo, where the rails pass it (round 9)
    # (the rails stop short of the beak servo, which goes in from the side between them)
    box('head top plate wide', (XHR - XLH - XLS - 0.0085, -RAIL, HEAD_PLATE_Z), (XHR - XLH - XLS - 0.0065, RAIL,
                                                                                   HEAD_PLATE_Z + 0.003)),
    # the left rail is the beak servo's rear plate now, inside the servo
    box('L head rail', (XHR - XLH - XLS - 0.0085, TAB_Y_ - BEAK_STACK - XL['box'][2] - 0.003, HEAD_Z + R.get('rail_z', 0.0075 * Q) + 0.004), ((XJ + XL['box'][0] / 2 - XL['shaft_x'] + 0.0006) if R.get('rail_steps') else XHR + 0.003, TAB_Y_ - BEAK_STACK - XL['box'][2], HEAD_PLATE_Z + 0.003)),
    # (past the beak servo's front the rail steps out, round the head roll's 15.5 mm bearing seat, which it stood in)
    *[box('L head rail front', (XJ + XL['box'][0] / 2 - XL['shaft_x'], 0.0165, HEAD_Z + R.get('rail_z', 0.0075 * Q) + 0.004), (XHR + 0.003, 0.0195, HEAD_PLATE_Z + 0.003)) for _ in range(bool(R.get('rail_steps')))],
    box('R head rail', (XHR - XLH - XLS - 0.0085, -RAIL, HEAD_Z + R.get('rail_z', 0.0075 * Q)), (XHR + 0.003, -RAIL + 0.003, HEAD_PLATE_Z + 0.003)),
    # the jaw's hinge screws go through the hood's wall (a spacer each, for the tab to turn on) into these bosses, and
    # are what holds the hood on
    *[g for sy in (-1,) for g in (       # the far side only: the left tab is on the horn
        box(f"{'L' if sy > 0 else 'R'} head hinge boss", (XJ - 0.005, min(sy * min(0.033 * Q, hood_b(XJ) - BOSS_IN - 0.0015), sy * (hood_b(XJ) - BOSS_IN)), ZJ - 0.003),
            (XJ + 0.005, max(sy * min(0.033 * Q, hood_b(XJ) - BOSS_IN - 0.0015), sy * (hood_b(XJ) - BOSS_IN)), ZJ + 0.007)),
        fast(cyl(f"{'L' if sy > 0 else 'R'} hinge screw", (XJ, sy * (TAB_Y_ + 0.0005), ZJ + 0.002), 'y', 0.0015, 0.006, DARK, 'decal'),
             'M3', 0.010, (XJ, sy * (TAB_Y_ + 0.003), ZJ + 0.002), (0, -sy, 0), pivot=True))],
    # (this bar ran straight through the Pi's board: no check looked at two parts of one link. It stops at a plate the Pi screws
    # to, through its 4 M2.5 corner holes, 58 x 23 mm, and the camera goes on the back of the face instead, ribbon to the Pi)
    *([box('Pi mount', (XHR + 0.0025, -0.006, EYE['z'] - 0.005), (PI_X, 0.006, EYE['z'] + 0.005))] if BUSES == 2 and not TWO_BOARDS else
      # (the bus board stands where this bar ran: 2 arms past it, outside where its plugs and leads go)
      [box('Pi mount', (XHR + 0.0025, 0.029, EYE['z'] - 0.005), (PI_X, 0.0325, EYE['z'] + 0.005)), box('Pi mount 2', (XHR + 0.0025, -0.0325, EYE['z'] - 0.005), (PI_X, -0.029, EYE['z'] + 0.005))]),
    box('Pi plate', (PI_X - 0.002, -0.0325, PI_Z - 0.015), (PI_X, 0.0325, PI_Z + 0.014)),
    *[box(f'Pi boss {k}', (PI_X - 0.0001, sy_ * 0.029 - 0.0025, PI_Z + sz_ * 0.0115 - 0.0025), (PI_X + 0.0028, sy_ * 0.029 + 0.0025, PI_Z + sz_ * 0.0115 + 0.0025), fillet=0.0005)
      for k, (sy_, sz_) in enumerate(((1, 1), (-1, 1), (1, -1), (-1, -1))) if not PI_BODY],
    *[fast(cyl(f'Pi screw {k}', (PI_X + 0.0028 + 0.0019, sy_ * 0.029, PI_Z + sz_ * 0.0115), 'x', 0.0019, 0.001, DARK, 'decal'), 'M2', 0.006,
           (PI_X + 0.0028 + 0.0014, sy_ * 0.029, PI_Z + sz_ * 0.0115), (-1, 0, 0)) for k, (sy_, sz_) in enumerate(((1, 1), (-1, 1), (1, -1), (-1, -1))) if not PI_BODY],
    *[box('Pi Zero 2 W', (PI_X + 0.0028, -0.0325, PI_Z - 0.015), (PI_X + 0.0028 + PI_T, 0.0325, PI_Z + 0.015), PCB,
          material='electronics', mass=0.011) for _ in range(not PI_BODY)],
    dict(box('camera', (FACE_X - 0.012, EYE['y'] - 0.012, EYE['z'] - 0.012), (FACE_X - 0.0015, EYE['y'] + 0.012, EYE['z'] + 0.012),
             PCB, material='electronics', mass=0.004), on_cover=True),
    # the 2 servo buses, up here by the Pi. Sized from their listings: FE-URT-1, 55.8 x 36.6 x 11.5 mm, on the Pi's UART, its servo power left
    # unconnected (the battery feeds the Feetech chain in the body: data and ground only come up the neck); U2D2, 48 x 18 x 15 mm, on the Pi's USB.
    # Both on foam tape. tools/... free-space map: nothing turns through these
    *[box(f'bus board rail {k}', (XHR - 0.0005, sy_ * 0.0244 - 0.003, BUS_Z - 0.002), (0.0930, sy_ * 0.0244 + 0.003, BUS_Z), fillet=0.0005) for k, sy_ in enumerate((1, -1)) if BUSES == 2 and not TWO_BOARDS],
    *[box(f'bus board post {k}', (XHR - 0.0005, sy_ * 0.0244 - 0.003, HEAD_PLATE_Z + 0.0028), (XHR + 0.003, sy_ * 0.0244 + 0.003, BUS_Z), fillet=0.0005) for k, sy_ in enumerate((1, -1)) if BUSES == 2 and not TWO_BOARDS],
    *[box('servo bus board', (0.0585, -0.0280, BUS_Z - 0.0002), (0.0951, 0.0280, BUS_Z + 0.0113), PCB, material='electronics', mass=0.015) for _ in range(BUSES == 2 and not TWO_BOARDS)],
    *([dict(box('U2D2', (XHR + 0.0048, -0.024, HEAD_Z + 0.0034), (XHR + 0.0228, 0.024, HEAD_Z + 0.0184), PCB, material='electronics', mass=0.009), on_cover=True)] if BUSES == 2 and not TWO_BOARDS else []),   # (flat, over what the head yaw servo's tray sweeps, under the Pi's mount)   # (on foam tape, clear of the horn's screw heads)
    *(bus_board() if (BUSES == 1 or TWO_BOARDS) and not PI_BODY else []),
    *(rear_board() if TWO_BOARDS else []),
    # the Pi's connectors, 2026-11-05: it was a plain box, so nothing said which way its USB edge faced, and no check saw a plug. That edge faces up: down has 0.5 mm to the head's base, up has 15 mm under the hood. Parts toward the face, so the microSD is at +y and the camera socket at -y.
    # Ports from Raspberry Pi's drawing: USB 41.4 and PWR 54 mm from the microSD's end. Right-angle plug bodies, 9 mm up and 15-19 mm along the edge, sizes from the listings' photos: USB leaves toward the bus board, PWR toward the regulator
    *[dict(box(n, (PI_X + 0.0028 + x0, y0, z0), (PI_X + 0.0028 + x1, y1, z1), '#D9D4C5', fillet=0.0003), material='electronics', mass=m, wire=True, exposed=True)
      for n, x0, x1, y0, y1, z0, z1, m in (*((('Pi USB plug', -0.0004, 0.0056, -0.0129, 0.0061, PI_Z + 0.015, PI_Z + 0.024, 0.002),
                                             ('Pi PWR plug', -0.0004, 0.0056, -0.0325, -0.0175, PI_Z + 0.015, PI_Z + 0.024, 0.002)) if not R.get('pi_uart') else ()),
                                           ('Pi camera ribbon', 0.0, 0.005, -0.0355 if TWO_BOARDS else -0.0360, -0.0325, PI_Z - 0.006, PI_Z + 0.006, 0.0002),
                                           ('Pi microSD', 0.0, 0.002, 0.0325, 0.0345, PI_Z - 0.006, PI_Z + 0.006, 0.0002)) if R.get('pi_ports_up')],
    # (the Pi's pins are the bottom edge now, pin 1 at the microSD's end. 3 wires to the Feetech board and 4 to the IMU, soldered on the parts' side, round the board's end, behind its plate)
    *[dict(box('Pi GPIO wires', (PI_X + 0.0028 + 0.0052, 0.0160, PI_Z - 0.0145), (PI_X + 0.0028 + 0.0072, 0.0335, PI_Z - 0.0115), '#D9D4C5', fillet=0.0003), material='electronics', mass=0.001, wire=True, exposed=True) for _ in range(TWO_BOARDS or bool(R.get('pi_uart')))],
    *jaw_recess(),
    loft('head', hood(), WHITE, open=(), along='x', clip=FRONT),
    cut(UNDER, 'head', 'upper bill'),
    # (pc_port, 2026-11-06, the user: "a small hole through which we can connect the pi0 to the pc, for loading models": straight over the Pi's micro USB data port,
    # 41.4 mm from its microSD end, which is free with the bus board on the UART. A plain micro USB cable goes down through it into the port. Wi-Fi does the same without a cable)
    *[cut(rbox('PC port hole', (PI_X + 0.0028 + 0.003, 0.0325 - 0.0414, PI_Z + 0.0385), (0.009, 0.013, 0.043), 0.0044, WHITE, 'struct'), 'head', 'upper bill') for _ in range(bool(R.get('pc_port')))],   # (a rounded slot: box() squares a printed part's corners)
    dict(plate('mouth roof', HEAD_Z - 0.0002, 0.0032, mouth_outline(MOUTH_X - 0.012), SOFT, 'shell', axis='z'), material='tpu'),
    cut(loft('face recess', arch_front(FACE_X, HEAD_FRONT + 0.01, RIM), WHITE, open=(), along='x'), 'head'),
    loft('upper bill', hood(grow=0.0006), ORANGE, open=(), along='x', clip=FRONT),
    cut(ABOVE_BILL, 'upper bill'),
    loft('face', arch_front(FACE_X - 0.0015, FACE_X + 0.0005, RIM + 0.0001), GREY, 'decal', open=(), along='x',
         clip=((0, 0, HEAD_Z + BILL_H), (0, 0, 1))),
    dict(loft('lower shell', band_secs(HEAD_BACK - 0.004, JAW_X0 - R.get('shell_gap', 0.011)), ORANGE, 'struct', open=(), along='x'),
         exposed=True),
    cut(box('lower shell opening', (HEAD_BACK + 0.010, -0.033 * Q, JAW_Z[0] - 0.01), (JAW_X0 - 0.010, 0.033 * Q, JAW_Z[1] + 0.01),
            WHITE, 'shell'), 'lower shell'),
    # (beside the beak servo, on its cradle's rear plate: 2026-11-02, they ran through its case)
    # the band's two arms tied straight up into the frame, so the head's base and its frame are one structure, as microduck's bottom shell is its base (the user, 2026-11-04: "why not the same part?"; it hung on one thin post)
    box('L base riser', (XHR - XLH - XLS - 0.0125, RAIL - 0.0055, JAW_Z[1] - 0.0015), (XHR - XLH - XLS - 0.0090, RAIL - 0.0025, HEAD_PLATE_Z + 0.003), exposed=True),
    box('L base riser top', (XHR - XLH - XLS - 0.0125, 0.020 * Q - 0.002, HEAD_PLATE_Z), (XHR - XLH - XLS - 0.0065, RAIL - 0.0025, HEAD_PLATE_Z + 0.003)),
    box('R base riser', (XHR - XLH - XLS - 0.0085, -RAIL, JAW_Z[1] - 0.0015), (XHR - XLH - XLS - 0.0025, -RAIL + 0.003, HEAD_Z + R.get('rail_z', 0.0075 * Q) + 0.002), exposed=True),
    # (2026-11-04: a voxel FEA of the head frame, 60 N on the back of the head: the left riser 45 MPa, the strip 23; the jaw servo stalled: the cradle's bars 8.3. The user: why not a plain plate? So: a plain end wall, the strip 5 mm tall, the riser 7 mm deep. 22, 13 and 0.9 MPa)
    *[box(nm, a_, b_, DARK, exposed=True) for nm, a_, b_ in ((n_, HO(a0), HO(b0)) for n_, a0, b0 in (('beak cradle end plain', (0.0104, 0.0147, 0.2502), (0.0134, 0.0402, 0.2602)),
                                                             ('head strip deep', (0.0149, -0.0310, 0.2632), (0.0169, 0.0142, 0.2662)),
                                                             ('L base riser back', (0.0074, 0.0285, 0.2347), (0.0110, 0.0315, 0.2652)))) if R.get('head_fixes')],
    # (the 5 V regulator for the Pi, off the board's green terminals: one power pair up the neck, not 2. A USB buck module, 35 x 17 x 8.5 mm by its listing, on edge on the base beside the Pi, its USB socket forward, for a right-angle plug turning in toward the Pi (a straight plug hits the face); mass a guess: weigh yours)
    *[dict(box('head 5 V regulator', *map(HO, R.get('reg_box', ((0.0660, -0.0420 - R.get('reg_out', 0), 0.2430), (0.1010, -0.0335 - R.get('reg_out', 0), 0.2600)))), PCB, material='electronics', mass=0.008), on_cover=True) for _ in range(bool(R.get('reg_head')))],
    # (the plug in its socket: its right angle turns in toward the Pi, in the 11 mm left to the back of the face. Clear of the camera ribbon's end, inboard)
    *[dict(box('head regulator plug', HO((0.0930, -0.0442, 0.2650)), HO((0.1040, -0.0357, 0.2810)), '#D9D4C5', fillet=0.0003), material='electronics', mass=0.002, wire=True, on_cover=True) for _ in range(TWO_BOARDS)],
    # (2026-11-05: the same for one board. Its socket stood 0.5 mm from the back of the face, so no plug fitted, right-angle or not. Found on the hybrid. So the module is turned round, its socket facing back, into the room behind it: a right-angle plug still needs 14 mm past the socket. Moving it back instead hits the hinge tab and the hood. Its input wires leave from the module's side, inboard)
    *[dict(box('head regulator plug', *map(HO, R['reg_plug']), '#D9D4C5', fillet=0.0003), material='electronics', mass=0.002, wire=True, on_cover=True) for _ in range('reg_plug' in R and not R.get('pi_uart'))],
    # (pi_uart: 2 wires soldered to the socket's VBUS and GND pins, bent along the module's side, inboard, and on to the Pi's pins 2 and 6)
    *[dict(box('head regulator wires', HO((R['reg_plug'][1][0] - 0.004, R['reg_plug'][1][1] - 0.0025, R['reg_plug'][0][2] + 0.001)), HO((R['reg_plug'][1][0], R['reg_plug'][1][1], R['reg_plug'][0][2] + 0.005)), '#D9D4C5', fillet=0.0003),
           material='electronics', mass=0.001, wire=True, on_cover=True) for _ in range('reg_plug' in R and bool(R.get('pi_uart')))],
    # (2026-11-05: the plug's back end stood 74 mm3 in the hood's step, on the inside. So that inner rib of the step now ends 1 mm short of the plug, at x = 51 mm. The head's skin isn't touched. The plug passes the hinge tab with only 0.2 mm)
    *[cut(box('head regulator plug room', HO((R['reg_plug'][0][0] - 0.001, -0.0433, R['reg_plug'][0][2] - 0.0005)), HO((0.0605, -0.0400, R['reg_plug'][1][2] + 0.0005)), WHITE), 'R hood step') for _ in range('reg_plug' in R)],
    # the head's base: one plate from the back band to the lip, as microduck's bottom shell, with an opening where the neck turns. The frame and the Pi's plate stand on it, the mouth pad is glued under it (the user: "one single piece from front to back")
    dict(plate('head base', HEAD_Z + 0.003, 0.0025, mouth_outline(HEAD_BACK + 0.008), DARK, 'struct', axis='z'), exposed=True),
    # notches for the jaw's tabs, which come up past the plate
    *[cut(box(f"{nm} base tab notch", (XJ - 0.014, sy_ * (TAB_Y_ - 0.0005) if sy_ > 0 else -0.08, HEAD_Z), (XJ + 0.024, 0.08 if sy_ > 0 else sy_ * (TAB_Y_ - 0.0005), HEAD_Z + 0.008), DARK), 'head base') for nm, sy_ in (('L', 1), ('R', -1))],
    cut(box('head base opening', (HEAD_BACK + 0.016, -0.0285 * Q, HEAD_Z), (PI_X - 0.004, 0.0285 * Q, HEAD_Z + 0.008), DARK), 'head base'),
    *[cut(cyl('head yaw round', (XHY, 0, HEAD_Z - 0.003), 'z', R['yaw_round'], 0.03, DARK), 'head base', 'head base wall', 'lower shell') for _ in range('yaw_round' in R)],
    # the band and the base, one shape: a wall all round the band's top, up into the plate (it hung from 2 small risers: the user, "why such small connections")
    dict(plate('head base wall', JAW_Z[1] - 0.0012, HEAD_Z + 0.0032 - JAW_Z[1] + 0.0012, mouth_outline(HEAD_BACK + 0.008), DARK, 'struct', axis='z'), exposed=True),
    cut(box('head base wall inside', (HEAD_BACK + 0.0105, -R.get('wall_in', 0.033 * Q - 0.0018), JAW_Z[1] - 0.003), (JAW_X0 + 0.04, R.get('wall_in', 0.033 * Q - 0.0018), HEAD_Z + 0.005), DARK), 'head base wall'),
    cut(box('head base wall front', (JAW_X0 - R.get('shell_gap', 0.011), -0.08, JAW_Z[1] - 0.003), (HEAD_FRONT + 0.03, 0.08, HEAD_Z + 0.005), DARK), 'head base wall'),
    # a real driver has to reach the head roll's 4 horn screws, before the Pi and the bus board go on
    cut(cyl('Pi plate driver hole', (PI_X - 0.001, 0, ZHR), 'x', 0.0095, 0.006, DARK), 'Pi plate', 'Pi board tab', 'Pi mount'),
    box('Pi base tab', (PI_X - 0.002, -0.0325, HEAD_Z + 0.005), (PI_X, 0.0325, PI_Z - 0.015 + 0.0002)),
    cyl('eye ring', (FACE_X + HK * 0.005, EYE['y'], EYE['z']), 'x', EYE['r'], HK * 0.0095, AMBER, 'decal'),
    # (printed black, a bore in it: the camera has to see out. It was a solid plug over the lens)
    cyl('eye lens', (FACE_X + HK * 0.0075, EYE['y'], EYE['z']), 'x', HK * 0.0086, HK * 0.008, LENS, 'decal'),
    cut(cyl('eye bore', (FACE_X + HK * 0.005, EYE['y'], EYE['z']), 'x', 0.0055, HK * 0.03, LENS, 'decal'), 'face', 'eye ring', 'eye lens'),
    *[cut(cyl('eye cup', (FACE_X + HK * 0.005 + HK * 0.00475 - 0.003 - 0.015, EYE['y'], EYE['z']), 'x', EYE['r'] - 0.0025, 0.030, LENS, 'decal'), 'eye ring', 'eye lens') for _ in range(bool(R.get('eye_cup')))],
    # the hood can't hang on its 2 hinge screws alone: on one axis, it would tip about them. A tab inside its back, 2 screws up
    # through the fixed lower band
    # (from the screw out to the hood's wall, as the hood widens forward)
    *[dict(plate(f'hood rear tab {k}', HEAD_Z + 0.0002, 0.0058, [(HRX2_[0], sy_ * 0.0258, 0.0026), (HRX2_[1], sy_ * 0.0258, 0.0026), (HRX2_[0], sy_ * (hood_b(HRX2_[0]) - 0.0026), 0.001), (HRX2_[1], sy_ * (hood_b(HRX2_[1]) - 0.0026), 0.001)], DARK, 'struct', axis='z'), exposed=True, on_cover=True)
      for k, sy_ in enumerate((1, -1))],
    *[fast(cyl(f'hood rear screw {k}', (HRX_, sy_ * 0.0258, HEAD_Z - 0.0026), 'z', 0.0019, 0.0012, DARK, 'decal'), 'M2', 0.008,
           (HRX_, sy_ * 0.0258, HEAD_Z - 0.0020), (0, 0, 1)) for k, sy_ in enumerate((1, -1))],
    box('sensor', (FACE_X, 0.030, EYE['z'] - 0.003), (FACE_X + 0.002, 0.042, EYE['z'] + 0.004), LENS, 'decal',
        fillet=0.0009),
])
# the lower band (round 5, the 10 mm grid): microduck's band runs flush all round under the upper bill:
# the jaw at the front (it opens), a fixed lower shell behind it; an orange tab on each side of the head carries the
# jaw's hinge
TAB_X = (XJ - 0.014, XJ + 0.014)
TAB_Y = TAB_Y_                                     # the hinge tab, in its step in the hood
link('beak_link', 0.03, [
    dict(loft('jaw', band_secs(JAW_X0, HEAD_FRONT - 0.0005), ORANGE, 'struct', open=(), along='x'), exposed=True),
    cut(box('jaw opening', (JAW_X0 - 0.01, -0.041 * Q, JAW_Z[0] - 0.01), (MOUTH_X, 0.041 * Q, JAW_Z[1] + 0.01),
            WHITE, 'shell'), 'jaw'),
    cut(plate('jaw tray', JAW_Z[1] - 0.006, 0.007, mouth_outline(MOUTH_X + 0.0015), ORANGE, 'struct', axis='z'), 'jaw'),
    dict(plate('jaw soft', JAW_Z[1] - 0.0061, 0.0051, mouth_outline(MOUTH_X + 0.0015), SOFT, 'struct', axis='z'), exposed=True,
         material='tpu', fill=0.4),                 # 40 % infill, as Open Duck Mini's TPU
    *[g for sy in (1, -1) for g in (
        # microduck's hinge tab: a rounded triangle standing on the band, its hub at the top on the hinge
        plate(f"{'L' if sy > 0 else 'R'} hinge tab", TAB_Y if sy > 0 else -TAB_Y - 0.004, 0.004,
              # (no rear lobe behind the jaw: it carried nothing, and needed its own groove in the hood. The user: "just cut that part")
              [(XJ, ZJ + 0.002, 0.011), (JAW_X0 + 0.0015, JAW_Z[0] + 0.0015, 0.0015),   # (its outline runs into the foot's corner: 4 mm here left a step and a notch at the join: the user)
               (TAB_X[1] + 0.004, JAW_Z[0] + 0.004, 0.004)], ORANGE, 'struct', exposed=True),
        box(f"{'L' if sy > 0 else 'R'} tab foot", (JAW_X0, min(sy * (TAB_Y + 0.004), sy * (hood_b(TAB_X[0]) - 0.002)),
                                                    JAW_Z[0]),
            (TAB_X[1], max(sy * (TAB_Y + 0.004), sy * (hood_b(TAB_X[0]) - 0.002)), JAW_Z[1]), ORANGE, exposed=True))],
])
joint('neck_pitch', 'trunk', 'neck_link', (XN, 0.013 * Q, ZN), 'y', RANGE.get('neck_pitch', (-0.15, 1.6)), R['neck'], out=+1, long='+z',
      case_in='child', rear_stack=R.get('rear_stack'), post=not NTOP)
joint('head_pitch', 'neck_link', 'head_pitch_link', (XN, -0.016 * Q, ZH), 'y', (-1.6, 1.6), R['head'], out=-1,
      long='-z', post=False, rear_stack=R.get('rear_stack'))                       # the head pitch bridge joins the fork's plates
joint('head_yaw', 'head_pitch_link', 'head_yaw_link', (XHY, 0, ZHY), 'z', (-2.5, 2.5), H2, out=+1, near=True,
      long='-x', fork=False, bearing=('horn',), face_t=0.001, sink=('horn',),
      seat_wall={'dynamixel': 0.0011}.get(RECIPE))   # (the 6703ZZ's seat, 0.5 mm wider, met the beak cradle at the head roll's end: its wall 1.1 mm, its outside 0.1 mm past the MR2216ZZ's)
#     a 1 mm sheet to the far holes, as microduck's: the
#                                                    beak servo turns over it
# (its case to the right, as microduck's, y -24.5 to 9.5: the left is the beak servo's)
joint('head_roll', 'head_yaw_link', 'head', (XHR, 0, ZHR), 'x', RANGE['head_roll'], H2, out=+1, long='-y', fork_at=90,
      bearing=R.get('roll_bearing', ('horn', 'rear')), sink=R.get('roll_sink', ('rear',)), rear_stack=R.get('rear_stack') if 'rear' not in R.get('roll_bearing', ('rear',)) else None)    # a bearing each side, as microduck's
# The jaw on the servo's horn, its shaft on the hinge, as microduck's (2026-11-04, the user: "get rid of the rod", "for simplicity". It had a crank, a rod and a lever from a servo 45 mm behind, which we took to be
# microduck's way: its model has none of them). 4 screws in the rear plate, no lid; a screw on the far side's hinge
joint('beak', 'head', 'beak_link', (XJ, TAB_Y_, ZJ), 'y', (0.0, R.get('beak_max', 0.4)), H2, out=+1, long='-x',
      sides=(), faces=('rear',), tray_full=True, near=True, fork=False, stack=R.get('beak_stack'))


# ---------------------------------------------------------------- mirror, servos

def mirror_geom(g):
    g = copy.deepcopy(g)
    g['name'] = g['name'].replace('L ', 'R ', 1) if g['name'].startswith('L ') else g['name']
    if 'targets' in g:
        g['targets'] = [t.replace('L ', 'R ', 1) if t.startswith('L ') else t for t in g['targets']]
    if 'contact' in g:
        g['contact']['c'][1] = -g['contact']['c'][1]
    if 'fast' in g:
        g['fast']['p'][1], g['fast']['d'][1] = -g['fast']['p'][1], -g['fast']['d'][1]
    if 'on_joint' in g:
        g['on_joint'] = mirror_name(g['on_joint'])
    for k in ('c', 'p0', 'p1'):
        if k in g:
            g[k][1] = -g[k][1]
    if g['kind'] == 'plate' and g.get('axis', 'y') == 'y':
        g['y0'] = -(g['y0'] + g['t'])
    elif g['kind'] == 'plate':                       # the outline's y: its first dim about x, its second about z
        k = 0 if g['axis'] == 'x' else 1
        for q in g['pts']:
            q[k] = -q[k]
    if g['kind'] == 'cyl' and not isinstance(g['axis'], str):
        g['axis'][1] = -g['axis'][1]
    if g['kind'] == 'loft':
        for s in g['sections']:
            s['cy'] = -s['cy']
    return g


def mirror_name(n):
    return 'R_' + n[2:] if n.startswith('L_') else n


HEAD_FRAME_ORANGE = {'beak cradle rim far', 'beak cradle rim near', 'beak cradle near post', 'rear board plate', 'rear board plate low', 'rear board plate low 2', 'rear board foot', 'rear board foot 2', 'rear board boss 0', 'rear board boss 1', 'rear board boss 2', 'rear board boss 3', 'beak horn seat boss', 'head frame', 'head top plate', 'head top plate wide', 'L head rail', 'L head rail front', 'R head rail', 'L head hinge boss', 'R head hinge boss', 'Pi mount', 'Pi mount 2', 'Pi board tab',
                     'bus board post 0', 'bus board post 1', 'bus board post 2', 'bus board post 3', 'Pi plate', 'Pi boss 0', 'Pi boss 1', 'Pi boss 2', 'Pi boss 3', 'bus board rail 0', 'bus board rail 1',
                     'lower shell', 'lower shell post', 'lower shell beam', 'head base', 'head base wall', 'Pi base tab', 'L base riser', 'L base riser top', 'R base riser', 'head roll horn boss shoulder',
                     'head roll rear boss shoulder', 'head roll fork horn plate', 'head roll fork rear plate', 'head roll fork post', 'beak cradle rear face', 'beak cradle end +', 'beak cradle end -', 'beak cradle side +', 'beak cradle end plain', 'head strip deep', 'L base riser back'}


# Each PWM servo's own lead to the PWM board, a star (2026-11-06, the user: "PWM servos can't be daisy-chained"): from where it leaves the case (its far end) along its
# chain, joint by joint, to the bonnet's plugs in the body, with a service loop at each joint it crosses. Its copper and PVC are counted on the links it lies on.
LEAD_KG_M = 0.008                                  # a 3-wire lead, 3 x 0.2 mm2 (24 AWG) of copper, 1.8 g/m each, and its PVC: an ESTIMATE, weigh one
LOOP = 0.03                                        # the service loop at each joint a lead crosses


def leads(links, joints):
    """{joint: (length m, [(link, length)])}, and the mass added to each link (on its first printed part, as 'add_mass')."""
    J = {j['child']: j for j in joints}
    plug = (PX_ + 0.0302, 0.0, BON_Z + 0.006)
    out = {}
    for j in joints:
        A_ = ACT[j['act']]
        f = servo_frame(j)
        p = at(f, A_['box'][0] / 2, 0.0, -A_['box'][2] / 2 + 0.004)    # the lead leaves the far end face, low
        ln = j['child'] if j.get('case_in') == 'child' else j['parent']
        segs = []
        while ln != 'trunk':
            jp = J[ln]
            q = jp['origin']
            segs.append((ln, math.dist(p, q) + LOOP))
            p, ln = q, jp['parent']
        segs.append(('trunk', math.dist(p, plug)))
        out[j['name']] = (sum(s for _, s in segs), segs)
    for name, (_, segs) in out.items():
        for ln, s in segs:
            g = next(g for g in links[ln]['geoms'] if g['role'] == 'struct' and 'fast' not in g and g.get('material') not in ('steel', 'electronics') and 'mass' not in g and g['color'] != ALU)
            g['add_mass'] = g.get('add_mass', 0.0) + s * LEAD_KG_M
            g['add_what'] = 'servo leads'
    return out


def build():
    links, joints = {}, []
    for n, l in LINKS.items():
        links[n] = copy.deepcopy(l)
        if n.startswith('L_'):
            m = copy.deepcopy(l)
            m['name'] = mirror_name(n)
            m['geoms'] = [mirror_geom(g) for g in l['geoms']]
            links[m['name']] = m
    for j in JOINTS:
        joints.append(copy.deepcopy(j))
        if j['name'].startswith('L_'):
            m = copy.deepcopy(j)
            m['name'], m['parent'], m['child'] = mirror_name(j['name']), mirror_name(j['parent']), mirror_name(j['child'])
            m['origin'][1] = -m['origin'][1]
            m['act_c'][1] = -m['act_c'][1]
            if m['axis'] in ('x', 'z'):                    # roll and yaw limits flip with the mirror
                m['lower'], m['upper'] = -j['upper'], -j['lower']
            if j['axis'] == 'y':                          # a horn facing +y on the left faces -y on the right
                m['out'] = -j['out']
            if j['long'][1] == 'y':
                m['long'] = ('-' if j['long'][0] == '+' else '+') + 'y'
            if 'y' not in (j['axis'], j['long'][1]):      # the servo frame's w is y: mirrored
                if j.get('fork_at') is not None:
                    m['fork_at'] = -j['fork_at']
                m['sides'] = [-v for v in j['sides']]
            joints.append(m)
    for j in joints:                                     # servo cases into the parent, horns onto the child
        case, horn = servo_geoms(j, BLACK if j['exposed'] else DARK)
        case_link, horn_link = (j['child'], j['parent']) if j.get('case_in') == 'child' else (j['parent'], j['child'])
        links[case_link]['geoms'].append(case)
        if not j['remote']:
            links[horn_link]['geoms'].append(horn)
        cradle, fork = mount_geoms(j)
        if j['act'].startswith(('XL330', 'XC330', 'HD1910')):      # the wires out of its side notches
            f, n = servo_frame(j), j['name'].replace('_', ' ')
            if BUSES == 1 and n[2:] in XL_REAR:
                A_ = ACT[j['act']]['box'][2] / 2
                for sg in (1, -1):
                    cradle = cradle + [dict(lbox(f"{n} cable plugs {'+' if sg > 0 else '-'}", f, XL_WIRES['u'], sorted((sg * 0.0062, sg * 0.0098)), (-A_ - 0.0045, -A_ + 0.0031), '#D9D4C5', 'struct', fillet=0.0004),
                                            material='electronics', mass=XL_WIRES['mass'], exposed=True, wire=True),
                                       cut(lbox(f"{n} wire window {'+' if sg > 0 else '-'}", f, (XL_WIRES['u'][0] - 0.0003, XL_WIRES['u'][1] + 0.0003), sorted((sg * 0.0059, sg * 0.0101)), (-A_ - 0.008, -A_ + 0.001), DARK), f'{n[0]} thigh plate')]
            else:
                cradle = cradle + [dict(lbox(f"{n} cable plugs {'+' if sg > 0 else '-'}", f, XL_WIRES['u'], sorted((sg * XL_WIRES['w'][0], sg * R.get('wire_w', XL_WIRES['w'][1]))), XL_WIRES['a'], '#D9D4C5', 'struct', fillet=0.0004),
                                        material='electronics', mass=XL_WIRES['mass'], exposed=True, wire=True) for sg in (1, -1)]
            # a groove for the wires where a plate lies against that side of the case
            for sg_, target in XL_GROOVES.get(n, ()) if RECIPE == 'hybrid' else XL_GROOVES_ALL.get(n[2:] if n[:2] in ('L ', 'R ') else n, ()):
                target = target.format(s=n[0])
                # (in a cover's 2 mm wall, a groove stops at the wires: it mustn't come through)
                cradle.append(cut(lbox(f"{n} wire groove {'+' if sg_ > 0 else '-'}", f, (XL_WIRES['u'][0] - 0.0003, XL_WIRES['u'][1] + 0.0003), sorted((sg_ * (XL_WIRES['w'][0] - 0.0003), sg_ * (R.get('wire_w', XL_WIRES['w'][1]) + (0.0002 if 'cover' in target else 0.003)))),
                                       (XL_WIRES['a'][0] - 0.0003, XL_WIRES['a'][1] + 0.0003), DARK), target))
        if 'steps' in ACT[j['act']] and (j['name'][:2] in ('L_', 'R_') or R.get('head_plugs')):                     # the cable plugs on the rear face, and the window behind them (PLUG): the hybrid's neck and head are XL/XC330s
            f, n, H = servo_frame(j), j['name'].replace('_', ' '), ACT[j['act']]['box'][2]
            s_ = n[0]
            if j['name'][:2] not in ('L_', 'R_'):
                # (no window where the rear plate stops short of them, the far end's only. The beak ends the head's chain, so one plug; its wires go back along the gap beside the roll servo, 7.5 mm as the ankle's)
                targets = {'neck pitch': (), 'head pitch': (), 'head yaw': ('head pitch bridge',), 'head roll': (), 'beak': ('L head rail', 'beak cradle rear face')}[n]
                cradle = cradle + [dict(lbox(f'{n} cable plugs', f, PLUG['u'], (0.0, PLUG['w']) if n == 'beak' else (-PLUG['w'], PLUG['w']), (PLUG['face'] - (0.0075 if n == 'beak' else PLUG['h']), PLUG['face']), '#D9D4C5', 'struct', fillet=0.0004),
                                        material='electronics', mass=PLUG['mass'] / (2 if n == 'beak' else 1), exposed=True)]
                w_ = PLUG['w'] + PLUG_WINDOW
                if n == 'beak' and R.get('beak_rail_pocket'):
                    cradle.append(cut(lbox('beak rail lump pocket', f, (0.0044, 0.018), (-0.0097, 0.0097), (-H / 2 - 0.0012, -0.0165), DARK), 'L head rail'))
                    # (the real case's raised strip on its rear face runs -4 to +22 mm along it, +/- 7 across, past the spec's lump; measured on SO-ARM100's mesh, after Fusion still showed 33 mm3 here)
                    cradle.append(cut(lbox('beak rear strip pocket', f, (-0.0075, 0.0228), (-0.0075, 0.0075), (-H / 2 - 0.0012, -0.0150), DARK), 'L head rail', 'beak cradle rear face'))
                    # (a 0.4 mm web stood between the plug window and the pocket for the lump, as on the hip roll: one opening)
                    cradle.append(cut(lbox('beak plug window web', f, (PLUG['u'][1], 0.0058), (-w_, w_), (-H / 2 - 0.003, -0.0135), DARK), 'L head rail', 'beak cradle rear face'))
                for t_ in targets:
                    cradle.append(cut(lbox(f'{n} plug window {t_}', f, (PLUG['u'][0] - PLUG_WINDOW, PLUG['u'][1] + PLUG_WINDOW), (-w_, w_), (-H / 2 - 0.012, -0.0135), DARK), t_))   # (both sockets on the beak too: the unused one's housing is still there, and stood in the plate: Fusion's box, 2026-11-05)
                links[case_link]['geoms'] += cradle
                links[horn_link]['geoms'] += fork
                continue
            target = {'hip yaw': 'top plate', 'hip roll': f'{n} cradle rear face', 'hip pitch': f'{s_} thigh plate', 'knee': f'{s_} thigh plate',
                      'ankle pitch': f'{s_} ankle stub plate'}[n[2:]]
            w_ = PLUG['w'] + PLUG_WINDOW
            cradle = cradle + [
                # (the ankle's: the plug's 7.5 mm only. Its wires turn up at once, tied to the rail, or the shoe's ear pinches them past 1 rad)
                dict(lbox(f'{n} cable plugs', f, PLUG['u'], (-PLUG['w'], PLUG['w']), (PLUG['face'] - (0.0075 if 'ankle' in n else PLUG['h']), PLUG['face']), '#D9D4C5', 'struct', fillet=0.0004),
                     material='electronics', mass=PLUG['mass'], exposed=True),
                cut(lbox(f'{n} plug window', f, (PLUG['u'][0] - PLUG_WINDOW, PLUG['u'][1] + PLUG_WINDOW), (-w_, w_), (-H / 2 - 0.012, -0.0135), DARK), target)]
            if R.get('head_plugs') and n[2:] == 'hip roll':   # (a 0.4 mm web was left between the plug window and the pocket for the case's lump: one opening)
                cradle = cradle + [cut(lbox(f'{n} plug window web', f, (PLUG['u'][1], 0.0058), (-w_, w_), (-H / 2 - 0.004, -0.0135), DARK), target)]
        links[case_link]['geoms'] += cradle
        links[horn_link]['geoms'] += fork
    # the two neck servos stand end to end, 0.5 mm apart: with their pockets' clearance, the plastic between them was a 0.2 mm sheet, which won't print. One pocket instead
    neck_g = {g['name']: g for g in links['neck_link']['geoms']}
    nk, hp = [g for g in neck_g.values() if g['role'] == 'actuator']
    lo_, hi_ = sorted((nk, hp), key=lambda g: g['c'][2])
    z0_, z1_ = lo_['c'][2] + lo_['size'][2] / 2 - 0.0003, hi_['c'][2] - hi_['size'][2] / 2 + 0.0003
    if z1_ - z0_ < 0.002:
        web = [n for n in neck_g if ' cradle ' in n and neck_g[n]['role'] == 'struct']
        links['neck_link']['geoms'].append(cut(box('neck servos pocket', (lo_['c'][0] - lo_['size'][0] / 2 - 0.00015, lo_['c'][1] - lo_['size'][1] / 2 - 0.00015, z0_), (lo_['c'][0] + lo_['size'][0] / 2 + 0.00015, lo_['c'][1] + lo_['size'][1] / 2 + 0.00015, z1_), DARK), *web))
    # a hole over every horn's centre screw: to reach it without taking the plate off, as the reference has (the user, 2026-11-04). Solid plastic there meant 4 screws out for one
    def on_plate(g, q):
        if g['role'] != 'struct' or g['color'] == ALU or g.get('material') in ('steel', 'electronics'):
            return False
        if g['kind'] == 'rbox':
            return all(abs(q[k] - g['c'][k]) <= g['size'][k] / 2 for k in range(3))
        if g['kind'] == 'cyl':
            k = 'xyz'.index(g['axis'])
            o = [i for i in range(3) if i != k]
            return abs(q[k] - g['c'][k]) <= g['h'] / 2 and math.hypot(q[o[0]] - g['c'][o[0]], q[o[1]] - g['c'][o[1]]) <= g['r']
        if g['kind'] == 'plate':
            ax_ = g.get('axis', 'y')
            k = 'xyz'.index(ax_)
            if not g['y0'] <= q[k] <= g['y0'] + g['t']:
                return False
            u, v = {'y': (q[0], q[2]), 'x': (q[1], q[2]), 'z': (q[0], q[1])}[ax_]
            pts = g['pts']
            return all((pts[i][0] - pts[i - 1][0]) * (v - pts[i - 1][1]) - (pts[i][1] - pts[i - 1][1]) * (u - pts[i - 1][0]) >= -1e-9 for i in range(len(pts)))
        return False
    for j in joints:
        if j['remote'] or not real_horn_hole(j):
            continue
        n = j['name'].replace('_', ' ')
        case_link, horn_link = (j['child'], j['parent']) if j.get('case_in') == 'child' else (j['parent'], j['child'])
        k = 'xyz'.index(j['axis'])
        top = list(j['origin'])
        q_ = list(j['origin'])
        q_[k] += j['out'] * 0.0005
        names = [g['name'] for g in links[horn_link]['geoms'] if g['role'] != 'cut' and (on_plate(g, q_) or g['name'] in (f'{n} horn seat boss', f'{n} horn boss shoulder'))]
        if names:
            c = list(j['origin'])
            c[k] += j['out'] * 0.003
            links[horn_link]['geoms'].append(cut(cyl(f'{n} horn screw access', c, j['axis'], 0.0032, 0.016, DARK, 'struct'), *names))
    # an HD-1910's rear horn has its own centre screw, its head 0.4 mm proud: a shallow recess in whatever lies on that horn (tools/real_fit.py found it, against Feetech's model)
    for j in joints:
        A_ = ACT[j['act']]
        if 'centre_screw' not in A_ or j['remote'] or not (j.get('fork', True) or 'ankle' in j['name']):
            continue
        n = j['name'].replace('_', ' ')
        k = 'xyz'.index(j['axis'])
        q_ = list(j['act_c'])
        q_[k] -= j['out'] * (A_['box'][2] / 2 + A_['horn_top'] + 0.0003)
        for ln_, l_ in links.items():
            names = [g['name'] for g in l_['geoms'] if g['role'] != 'cut' and on_plate(g, q_)]
            if names:
                c = list(q_)
                c[k] += j['out'] * 0.0002                      # 0.3 mm deeper than the head stands, and into the horn's own air on the other side
                l_['geoms'].append(cut(cyl(f'{n} rear screw recess {ln_}', c, j['axis'], 0.0027, 0.0010, DARK, 'struct'), *names))
    # its rear tube, where the horn comes off: a pocket for 2.5 mm of tube left a 0.35 mm floor in a 3 mm plate, which no printer holds. A hole right through instead
    for j in joints:
        A_ = ACT[j['act']]
        if 'rear_stub' not in A_ or 'ankle' in j['name'] or j['name'][2:].replace('_', ' ') in XL_REAR or (j.get('fork', True) and not j['remote']):   # (a thigh's plate has the cover's post over the tube, so there the pocket has a floor: the post, widened to take in its corners)
           
            continue
        n, f, H_ = j['name'].replace('_', ' '), servo_frame(j), A_['box'][2]
        case_link = j['child'] if j.get('case_in') == 'child' else j['parent']
        names = [g['name'] for g in links[case_link]['geoms'] if g['role'] != 'cut' and on_plate(g, at(f, -A_['shaft_x'], 0, -H_ / 2 - 0.0028))]
        if names:
            links[case_link]['geoms'].append(cut(lbox(f'{n} rear tube hole', f, (-A_['shaft_x'] - 0.0028, -A_['shaft_x'] + 0.0028), (-0.0028, 0.0028), (-H_ / 2 - 0.004, -H_ / 2 + 0.0002), DARK), *names))
    for l in links.values():
        for g in l['geoms']:
            if g['kind'] == 'rbox' and min(g['size']) <= 0:   # (another variant's numbers on a smaller body: it hung the checks for half an hour)
                raise SystemExit(f"{g['name']}: a box with no size, {g['size']}: a number here doesn't fit this variant")
            if g['role'] != 'cut':
                g['material'] = material_of(g)
            if 'fast' in g:                              # a screw at what it weighs: 190 on the small duck, about 60 g (2026-11-04, the user: count them). They were a PLA disc each
                g['material'], g['mass'] = 'steel', screw_mass(g['fast'])
    # the head frame is one print in one filament: the rim shows below the hood, the rest is hidden under it (2026-11-04, the user chose all orange over a filament swap)
    for lk in links.values():
        for g in lk['geoms']:
            if (g['name'] in HEAD_FRAME_ORANGE or (R.get('head_plugs') and g['name'] == 'head roll rear seat boss') or (PI_BODY and g['name'] == 'head roll horn seat boss')) and g['role'] != 'cut':
                g['color'] = ORANGE
    lead = leads(links, joints) if R.get('pwm') else None
    return dict(recipe=RECIPE, recipe_limits={k: R[k] for k in ('mass_budget_kg', 'target_tol', 'cannot', 'brief') if k in R},
                **({'leads': {k: dict(length_m=round(v[0], 3), links={ln: round(s, 3) for ln, s in v[1]}) for k, v in lead.items()}, 'lead_kg_m': LEAD_KG_M, 'wiring_allowance': 0.01} if lead else {}),
                frame='metres; X forward, Y left, Z up; floor z=0; zero pose standing, legs straight, head level',
                actuators=ACT, bearings=BEARINGS, materials=MATERIALS, links=links, joints=joints, root='trunk',
                beak=dict(crank_c=[XBK, ZBK], pin=list(BEAK_PIN), jaw_pin=list(JAW_PIN), jaw_c=[XJ, ZJ]))


if __name__ == '__main__':
    robot = build()
    out = os.path.join(HERE, 'robot.json' if RECIPE == 'hybrid' else f'robot_{RECIPE}.json')
    with open(out, 'w', encoding='utf-8') as f:
        json.dump(robot, f, indent=1)
    print(f"{len(robot['links'])} links, {len(robot['joints'])} joints -> {out} (masses: python -I tools/mass_model.py)")
