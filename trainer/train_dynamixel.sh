#!/bin/bash
# Train the all-Dynamixel duck on a machine with a big GPU (docs/TRAINING.md): walk, get up, head roll, one at a time, then export to pi/models/.
# Run from the trainer's folder, with its environment active, after docs/TRAINING.md's set-up. Not run yet: the first full run will be the test of it.
#   bash ../duck-droids/trainer/train_dynamixel.sh [walk] [standup] [roll]
set -e
export WANDB_MODE=offline PYTHONUTF8=1
ENVS=${ENVS:-4096}
OUT=../duck-droids/pi/models
mkdir -p "$OUT"
tasks=${*:-walk standup roll}
run() {   # name, task, log folder
  train "$2" --env.scene.num-envs "$ENVS"
  last=$(ls -t "logs/rsl_rl/$3"/*/model_*.pt | head -1)
  echo "$1: $last"
  python scripts/export.py "$2" --checkpoint-file "$last" --onnx-file "$OUT/$1.onnx" || echo "export $1 by hand: see src/mjlab_microduck/export.py"
}
for t in $tasks; do
  case $t in
    walk)    run walk Mjlab-Velocity-Flat-Milli-Dynamixel milli_dynamixel; python scripts/milli_eval.py --recipe dynamixel --envs 512 ;;
    standup) run standup Mjlab-StandUp-Flat-Milli-Dynamixel milli_dynamixel_standup; python scripts/milli_skill_eval.py standup --recipe dynamixel ;;
    roll)    run roll Mjlab-Roulade-Flat-Milli-Dynamixel milli_dynamixel_roulade; python scripts/milli_skill_eval.py roulade --recipe dynamixel ;;
  esac
done
