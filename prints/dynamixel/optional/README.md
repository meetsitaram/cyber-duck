# Optional printed idler

In place of ROBOTIS's FPX330-H101, on the rear of an XL330. The duck uses 5. Made by `tools/idler_print.py`: our own simple shapes, sized to the interface measured from ROBOTIS's model.

| File | Size | Print |
|---|---|---|
| `idler_wheel.stl` | 16 mm across, 4 mm thick | flat face down, no supports |
| `idler_bushing.stl` | 6.8 mm across, 4.4 mm long | flange down, no supports |

**Nothing here has been printed.** The bushing and wheel are a plastic bearing, with a 0.4 mm gap as drawn because printed holes come out small. Print one and check:

1. The wheel turns on the bushing with little play. If not, scale the bore in `tools/idler_print.py`.
2. An M2.5 x 8 mm self-tapping screw bites in the servo's rear hole, and clamps the bushing without stopping the wheel.
3. The M2 screws bite in the wheel's 1.6 mm holes. Only 1.2 mm of plastic is outside each hole, so stop as soon as they're snug.

Use PETG or nylon if you have it: PLA creeps and wears. If an idler carries a walking joint, buy the ROBOTIS ones once you can.
