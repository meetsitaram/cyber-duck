---
name: duck-at-size
description: Design, check, train and iterate a small open biped duck at any height the user names, in the milli / duck-droids repo, choosing the servos and working in rounds with the user until it walks, gets up and tries the head roll in simulation. Use it whenever someone asks for a duck, Microduck, Mini Duck or biped of a given size or on given servos, asks whether a size or servo can work, wants a new servo recipe, or wants to compare recipes, even if they only give a height ("make me a 40 cm duck") and don't mention this repo or the word recipe.
---

**Phases:** this skill sizes a duck and iterates it in simulation. For the build itself, parts, detail pass, CAD, prints and assembly guide, follow the phases in `droid-from-spec`.

# A duck at any size

The user names a size. You drive the whole design from there, in rounds, as echo's humanoid was built: one change, the checks, pictures, a review, then the next. The tools build, check and measure. They cannot judge whether a duck looks right, whether a score is real, or whether a trade-off is acceptable. You and the user do.

Read `LESSONS.md` first, and the `droid-from-spec` skill for the general method and its eight rules, which all apply here. `docs/TRAINING.md` covers the trainer's setup.

"Any size" has limits, so say them early. A servo's case sets a smallest duck on that servo, and with the same servo the torque a joint needs grows with about the fourth power of size. What the skill promises is a duck at the size asked for, or a clear reason why not, with the nearest size that works.

## A second variant of a built design

`MILLI_RECIPE=<recipe>` gives each variant its own spec, reports, prints and CAD. Put its own choices in `RECIPES` (boards, battery, covers), not in shared code: the first variant's numbers will not fit a smaller body. Then the same suites, in order: `spec/robot.py`, `tools/checks.py`, `tools/assembly.py`, `tools/buildable.py`, `tools/beak_linkage.py`, `tools/printable.py`, `tools/export_prints.py`, `tools/verify_fusion.py write`, `cad/fusion/full_build.py`, `cad/fusion/export_step.py`, `tools/verify_fusion.py compare`, and look at the pictures from every side. Ask what parts are already on the shelf: it decides what can be built first. A change to a shared part's model means rebuilding every variant that uses it.

## Round 0: agree the goal

Ask only what you can't default:
- **Height**, and what must match: Microduck's look, a price, servos they own.
- **Which skills matter:** walking, getting up, the head roll. A duck that only has to walk can use a much lighter neck.

Then get the facts:

```bash
python tools/new_duck.py --height-cm 34 --compare
```

This tries every servo in the catalog at that height and prints the height reached, width, mass, roll margin and failing checks. Show the user that table with your pick, and why. A servo not in the catalog goes in `spec/actuators.json` from the maker's drawing (case, shaft offset, horn), with `KP` in `tools/make_rl_model.py` and a line in `CATALOG`. Look its numbers up, state their source, and mark what's estimated. Sellers and makers often disagree: quote both.

Before picking any mechanism or part new to this size, look for robots that use it at that size and load, and name them when proposing it. If none turn up, say so: it's a warning sign, and the user's call. A roller-chain knee passed its torque numbers on a 1.8 m duck, but a search only found chain on small robots.

Check how the reference runs its parts before quoting a datasheet: Microduck runs its XL330s past the maker's voltage limit, at nearly twice the 5 V stall torque.

## Each design round

1. **One change**, in `spec/recipes/<name>.json` or, for something every recipe needs, `spec/robot.py`, with the reason in a comment. Generate with `MILLI_RECIPE=<name> python -I spec/robot.py`. Never hand-edit a `robot_*.json`.
2. **Checks:** `MILLI_RECIPE=<name> python -I tools/checks.py --out reports/<name>/checks.md`. Fix the design, not the check. A limit that can't be met goes in the recipe's `mass_budget_kg`, `target_tol` or `cannot`, and is told to the user and written in the README. It never gets dropped quietly.
3. **Pictures:** render it beside the other recipes at one scale (`tools/render_recipes.py`; add it to `RECIPES` there). Look at them yourself first, then send them. Take every comment on the look literally.
4. **Review:** wait for the user's reaction on anything that changes the look.
5. **Record:** a `LESSONS.md` row for each mistake, in the same step, unasked.

Which number fixes which failure:

| Failure | Change |
|---|---|
| Beak servo through the hood, top plate outside the skin | `HK` up, or `beak_x`, `plate_back` |
| Hinge boss outside the skin | `boss_in` |
| Neck and head pitch cases meet | `neck_rise` up |
| Ankle blocked by the shoe or shin | `za` up, `collars: false`, `cap`, `rib_up`; then the range |
| Knee or ankle range short | `ranges`, as far as the sweep allows; Microduck's are about +-1.57 |
| Head roll blocked | `rail`, or a smaller `head_roll` range |
| Parts in two pieces after a shorter case | `short`: the case's difference along its shaft |
| Battery doesn't fit under the plate | `battery` |
| Proportions out | `HK`, `K`, then `target_tol`, declared |

Too small for its servo: stop shrinking, and tell the user the smallest size reached, what stops it, and which change or servo could go further. Don't produce a duck that only passes because limits were loosened.

## Before any long run

When the checks pass and the user likes the look:

```bash
python tools/new_duck.py --height-cm 34 --servo HD1910_C001 --name d34 --finish
```

This writes the trainer's models and measured reset heights, and registers `Mjlab-Velocity-Flat-Milli-D34`, `Mjlab-StandUp-Flat-Milli-D34` and `Mjlab-Roulade-Flat-Milli-D34`. Then, in minutes rather than hours:

1. **Standing:** `python scripts/milli_sanity.py Mjlab-Velocity-Flat-Milli-D34` in the trainer. Low-gain servos may fall with zero actions, as Microduck's own do by 2 s. Falling at once means a wrong gain or stance.
2. **Scripted flip:** `python tools/flip_search.py <trainer>/milli_d34_contact OUT.npz`, then `MILLI_RECIPE=d34 python tools/check_motion.py OUT.npz 2`. This tells a weak robot from a weak policy. It can show a move is possible, never that it's impossible.
3. **Audit the reused recipe's numbers** for anything sized for another robot, as `docs/TRAINING.md` lists.

If the flip can't get over and the user wants the roll, go back to a design round: a stronger neck or a lighter body beats more training.

## Training rounds

Run at most three trainings at once on one GPU, and count CPU workers against the machine too: overload has reset the display driver and killed every run. Check iteration speed after adding a job.

```bash
WANDB_MODE=offline PYTHONUTF8=1 train Mjlab-Velocity-Flat-Milli-D34 --env.scene.num-envs 4096
```

**Test early, don't wait for the end.** At each 500–1000 iterations:

```bash
python scripts/milli_eval.py CKPT.pt --recipe d34 --envs 64
```

```bash
python scripts/milli_skill_eval.py standup CKPT.pt --recipe d34 --trajdir DIR
```

A good score is not a working skill. Every checkpoint you report has to survive this list, as each item has caught a real cheat here:

| Look at | The cheat it caught |
|---|---|
| Each joint's position, torque and speed | Hips pressed on their stops at stall for a straight walk; the neck the same in stand-up |
| Each robot's start-to-end speed and heading, not averages | Circles counted as forward speed |
| Yaw rate per step | A twist hidden in time averages |
| Every command: slow, backward, sideways, turn, stand | A walk that only works fast and forward |
| `tools/check_motion.py`, with `MILLI_RECIPE` set | Rising by passing feet through feet |
| The start of the run, from rest on the floor | Standing up mid-fall from a reset in the air |
| A rise that looks too fast | Any of the above |

A new penalty moves the exploit, so re-read everything after each reward change. Fix what you find in every task, then restart what it affects. Tell the user what was wrong, what the numbers really are, and what restarting costs.

Render with `MILLI_RECIPE=<name> python tools/render_walk.py WALK.npz OUT.mp4 SHEET.png`, look at the sheet, and send videos with the recipe, the checkpoint and "simulation only" in the caption.

**Far above the servo sizes (a metre or more):** a new spec, not a recipe: `spec/big_duck.py` is the example. Real parts: actuators, plate frames, bearings, screws, panels under the printer's bed. Then expect training to need more than scaling:

- `scale_for_big` scales what has units, and that isn't enough: correct numbers can leave a reward undiscoverable. At iterations 250-750, compare each reward term with a robot that learned. A term at zero has no path to it. The big duck needed a stepping clock.
- From the first checkpoint that moves, test time at each joint's limits, in stance and in swing (`scripts/milli_limits.py`). A torque that doesn't fall with mass is a joint pressing on a stop.
- Give every joint a stop penalty, and penalise sustained load over the rated torque, not only near stall.
- Before calling a joint weak or naming a cause, measure it, and compare it with robots of its own class from their own models, by weight x leg length. A small servo robot is far stronger for its size and is no baseline.
- Size any limited-range mechanism, as a crank, from this robot's own motion, not a smaller robot's.
- When a walk is bad, ask what the mass, mechanism and stance each contribute before more rewards. Weight off the top first: `tools/leg_limits.py`.

## Finishing

- A comparison table from one set of tests across recipes, with the mistakes kept in.
- The README picture, numbers and status, copied from the reports.
- Say what isn't covered: simulation only, estimated masses, unmeasured servos, policies not tried on real parts. Suggest testing one real servo before buying a set.
- Commit and push only when asked. On Windows, keep working files on short paths.

## What the tools don't do yet

The tools are built around the hybrid, so tell the user these are open for a new recipe, and do them by hand or extend the tool:
- `tools/assembly.py`, `tools/buildable.py`, `tools/make_docs.py` and the print export name its parts and servos. Until they're generalised, a new recipe has no tested assembly order, parts list or printable files.
- Fusion: `cad/` builds from the spec, but check that it handles a new recipe before promising CAD. It needs the Fusion connection.
- `new_duck.py` only reaches a height by scaling `K`, the legs and body, as the neck's numbers are tied to its servo. At 34 cm on the XL330 and HD-1910 it reached the height with 4-5 checks failing: hips too high, body and feet too big, and ankles blocked at about 1.1 rad. Expect a design round for any height well above a layout's smallest: scale the neck with it and rework the ankle. Nothing reaches below 29.5 cm yet.
- `new_duck.py` scales from the three layouts in `spec/robot.py`. Well outside about 26–45 cm, expect the layout's fixed numbers, or the structure itself, to need real design work.
- Other people's duck files often restrict commercial use, or have no licence. Use published specs, not their parts or models, unless their licence allows it.
