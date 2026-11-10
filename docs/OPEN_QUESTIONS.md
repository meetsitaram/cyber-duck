# Open questions

Questions that still need an answer, from a measurement, a maker or you. Where one had to be settled to keep designing, the choice made is written here, so it can be reviewed later.

## Chosen for now, to review

| Duck | Question | Chosen | Why | How to settle it |
|---|---|---|---|---|
| Big | How deep is the RS04's M5 output thread: 3.5 or 6.5 mm? | **3.5 mm.** About 3 mm of thread engaged, never reaching the bottom, and the 3 dowel pins carry the torque | It's what RobStride's STEP file shows (blind at z +22 mm). 6.5 mm is the distance from the floor to the pin tips, the figure the spec had. Designing for the shallower hole is safe if the real one is deeper, but not the other way round | Measure a real RS04 with a depth gauge, or ask RobStride. If it's 6.5 mm, longer screws can be used. If it's 3.5 mm, ask what the screws and pins are each rated for. 9 M5 screws with under 4 turns each are weak in pull-out |
| Hybrid | The head's servos at 5 V through a UBEC, or straight from 2S? | **5 V** | Inside ROBOTIS's 6 V limit, and the parts and model are already built this way | Bench-test one XL330 and one XC330 on a full 2S against 5 V, for temperature and life. See [WIRING.md](hybrid/WIRING.md#todo-review-the-heads-voltage) |
| HD-1910 | Keep Feetech's own HD-1910 model inside `cad/duck-droids-hd1910.f3d` and `.step`? It came with no licence terms | **Keep, for now** (2026-11-05) | It shows the real servo, and THIRD_PARTY_NOTICES says where it came from | Ask Feetech for terms. If they object, re-export without it: the spec's own servo shape can stand in |

## Waiting for you

| Duck | Question |
|---|---|
| Big | Many frame joints are M4 screws tapped into the edge of 8 mm plate: the hip pitch bridge, head pitch yokes, head roll cradle, head rails, beak mount and tray, shin top. Accept them, or go to angle brackets or thicker plate? Nothing has checked their strength |
| Big | The shin is a 5 mm 6061 tube with one welded seam. Accept a weld there? |
| Big | The hip roll yoke and its standoff ring are one machined billet part, as there's no room for screws. Acceptable to machine? |
| Big | 14 of the 22 actuator harnesses don't reach the boards on the deck: even the shortest route past the hips needs about 420 mm against 350. A CAN hub and power block in each leg, or extension cables? See [CABLES.md](big/CABLES.md) |
| Big | The knee's two RS04s each drive their own crank and rod into one lever: two linkages constrain the joint twice, and unequal rods preload each other. Keep it with adjustable rods, put both RS04s on one crank shaft with one rod, or rebuild the hip roll and pitch as a differential? See [PAIRED_ACTUATORS.md](big/PAIRED_ACTUATORS.md) |
| Big | Paired actuators' control: one joint controller, torque split between the pair, a small anti-backlash bias, a joint calibration and fault check at start. Designed in [PAIRED_ACTUATORS.md](big/PAIRED_ACTUATORS.md), not written or tested. Read Robertz et al. (ICRA 2010) in full before citing its figures |
| Big | Delete the old `big assembly` document in the Fusion hub, now that `big assembly real` replaces it? |
| All Feetech | Bench-test one STS3215 at a full 8.4 V before fitting? Sellers disagree on whether the 7.4 V servo takes it |
| All Feetech | A fuse at the XT30? To decide once walking current is measured |

## Design changes to make (2026-11-06)

The all-Dynamixel BOM already lists these parts. **All-Dynamixel: done on branch `dynamixel-6703-uart`** (spec, print files, BOM, assembly, wiring drawing and WIRING.md, Pi setup), with the checks in CLAUDE.md at 0 failing. Rebuilt in Fusion on 2026-11-09, and every print file is from those bodies. Still to do there: the reveal, the 5-minute build film and `parts_layout.png`, redrawn from the new bodies. The hd1910 and the hybrid's head still use the MR2216ZZ.

- **Horn bearings: MR2216ZZ (16 x 22 x 4 mm) to 6703ZZ (17 x 23 x 4 mm).** Amazon had no 16 x 22 x 4 mm bearings until the end of the month. The uxcell 6703ZZ (B082PSB4DZ, 5 for $6.29) is in stock. In `spec/robot.py`, `BEARINGS['XL330']` becomes `d=0.017, D=0.023`, and the mass is about 6.4 g, from uxcell's 32 g for 5.
  - The 17 mm bore is 1 mm over the 16 mm horn, so the inner race sits on the printed boss alone, not half on the horn as before. Check that the boss holds it, and test the fit on the first printed joint.
  - The seats grow 0.5 mm in radius. Rerun the clash and wall checks.
  - The other XL330 ducks change too: hd1910, and the hybrid's head. Update their BOMs (`tools/bom_hd1910.py`, `tools/make_docs.py`).
- **No right-angle USB cables.** They're hard to find.
  - **Power:** solder the 5 V regulator's output to the Pi's 5 V and GND pins (2 and 6), or use a 5 V buck with wire outputs.
  - **Data:** connect the Pi's UART (GPIO 14, GPIO 15 and GND) to the Waveshare board's TX, RX and GND header, with its jumper set to UART. Use the Pi's full UART, not the mini one: set `dtoverlay=disable-bt` or `miniuart-bt`.
  - Redraw the leads in the head and update WIRING.md and the wiring pictures.
  - The hybrid has two buses, and the Pi Zero 2 W has only one full UART, so one of its boards may stay on USB.

## To do: make the all-Dynamixel parts printable (2026-11-09)

The first real prints showed the parts were never validated in a slicer. Some have almost nothing on the bed, or print a large plate on thin legs. Design improvements to make later; none is done yet.

First layer is the part's area 0.2 mm above the bed, measured on the Fusion STLs. "Best of 6" is the most any of the six ways of laying it gives.

| Part | First layer now | Best of 6 | What's wrong | What to do |
|---|---|---|---|---|
| `thigh_frame` (L, R) | 62 mm2 | 62 mm2 | A 2225 mm2 plate 28 mm up, on four thin prongs: Bambu Studio on a P1S gives 8.9 g of part and 6.5 g of support, 58 min. Printing it plate side down leaves it on two small bosses. | Redesign for a face on the bed, or split the plate from the prongs |
| `shin` (L, R) | 126 mm2 | 137 mm2 | Only rounded edges reach the bed. Stood on tree supports for the first print: its upright plate has layers across the load, and sideways holes. | Add a flat on an outer side that doesn't mate |
| `chassis` | 44 mm2 | 178 mm2 | No flat face | Add a flat |
| `shoe` (L, R) | 44 mm2 | 123 mm2, with far more overhang | No flat face | Add a flat |
| `sole` (L, R, TPU) | 87 mm2 | 1286 mm2 | Exported on its edge | Lay it on its big face |
| `neck_column` | 83 mm2 | 250 mm2, with more overhang | Little on the bed | Lay it on its side, or add a flat |

**Servo screws (found on the first assembly, 2026-11-09).** The servo box has black pan-head self-tapping screws, measured under the head: M2 x 6 (7.3 mm overall) and M2 x 8, and some shorter ones (5.1 mm overall, about 3.6 mm under the head; what they're for isn't known). The M2 x 6 fitted the original shoe with no recess. A 5 mm recess for the short screws was tried, fitted too, and was taken back out, so the box's M2 x 6 and x 8 go in every horn and case.

- The kit screws sold as "M2*6" are flat heads whose length includes the head, with a pointed tip: not a substitute for the horns.
- The horn and idler holes are 1.6 mm wide and 2.9 mm deep (ROBOTIS's models). The M2 x 6 through the 3 mm plates at the neck pitch, head pitch and beak reaches about 3.0 mm: tighten gently, and confirm on the real parts.
- Done: horn screw through-holes are 2.4 mm, and the ankle rear plate has a 0.8 mm seat for its flat heads, so the small bearing can seat. Neither is tested on a real part yet.
- To do: teach `tools/assembly.py` to fail a screw that bottoms out, and check the 0.8-0.9 mm of plastic under the head yaw servo's case screws.

**Ankle's far side, for the next version (2026-11-09, the user).** The stub plate, stub and 6700ZZ bearing aren't needed: the plate bolts the servo's idler to the shin, so the idler can't turn, and the stub and bearing then add back the same free joint further out. Bolt the shoe's inner ear straight to the idler, as the Koch arm's forks are. That removes the stub, the bearing, the 8 flat-head screws and the rear plate's seat, and could narrow the ankle by a few mm.

- The shin needn't reach the idler at all. It only goes down there to carry the stub and to use the idler as 4 more screw holes. Stop it at the servo case and use the case's near holes as well as the far ones, if the cable plugs beside them leave room.
- The shoe can then be one piece, ears and all, and the separate inner ear and its screws go. It was only split because a one-piece shoe can't go on over the stub (LESSONS.md). Check the servo with both horns slides between the ears (Koch's forks leave 29.4 mm for its 29 mm), and how the U prints.
- The shin then needs another hold on the servo near the idler: those 4 flat-heads are most of what ties the servo's rear to the shin, with only 2 other screws.
- The idler's plastic bushing would carry the walking load. No figure for it: bench-test one joint.

**Shin to ankle servo: hold it on both sides (2026-11-09, from the first real shin).** The shin holds the ankle servo on its rear face only, and flexes that way. Add a second side: a yoke round the servo's far end, down its outer face to the case holes there, clear of the shoe's ear over the ankle's range. Design this together with the ankle change above: that one takes away the 4 screws into the idler, which would make this flex worse.

**Feet that print flat (2026-11-09, the user).** Make the shoe's underside and the sole's top flat, screwed together, so each prints on its flat face.

- Remove the rails along the shoe's bottom edges (z 7.9 to 11.9 mm) and the sole's matching channels. The seam at the foot's sides moves up about 4 mm, which changes the look a little, and only the screws then locate the two parts.
- Remove the curved trough in the sole. Measured over the ankle's whole range (+/-88 degrees), the ankle servo's lowest point is z 15.3 mm, 3.4 mm above the sole's flat top at z 11.9. The trough is only 0.6 mm deep, and isn't needed.
- Keep the opening through the shoe's plate: its top is at z 18.9, above the servo's 15.3. It's a through-slot, so it doesn't affect printing.

The checks to fix, so this can't pass again:

- `tools/printable.py` scores the offline shapes, before Fusion rounds them. Check the final STLs in `tools/audit_prints.py`, which today only tests that they fit the bed.
- Contact counts any downward face within 0.3 mm of the bed. Measure the first layer instead.
- The orientation is the least overhang, and falls back silently to all six when none has enough contact. Fail instead.
- Slice every part in a real slicer. Bambu Studio's command line works with the P1S profile (about 2 s a part); a batch over all parts and orientations hasn't run successfully yet.

## Settled

- **Big, RS03 mounting:** each RS03 now uses its 8 free front holes, through a 6 mm ring with countersunk M4 screws, as well as the 4 rear ones.

## Measure on the first parts

- **Big:** the RS04's mass (no file gives one; sellers say 1.42 and 1.5 kg), each CAN bus's terminators (measure between H and L with the power off), and walking current on a current-limited supply before any pack goes near an actuator.
- **Small ducks:** walking current. The race-to-bracket clashes are mostly the drawn press fit, so check that a race seats on the first printed joint.
