#!/bin/bash
# Cyber Duck's build in small runs: each process carves two pieces, caches them and exits, so its memory goes back to
# the PC; then the transformation video. Every run has the 6 GB cap (tools/memcap.py) and a timeout.
cd "$(dirname "$0")/.." || exit 1
export MILLI_RECIPE=${MILLI_RECIPE:-dynamixel} CYBER_PIECES_PER_RUN=2
log=reports/$MILLI_RECIPE/cyber_duck/build.log
mkdir -p "$(dirname "$log")"; : > "$log"
for i in 1 2 3 4 5 6; do
  timeout 1500 python tools/cyber_duck.py >> "$log" 2>&1; rc=$?
  [ $rc -eq 3 ] && continue
  echo "build exit $rc" >> "$log"; [ $rc -ne 0 ] && exit $rc; break
done
# video: run separately > "reports/$MILLI_RECIPE/cyber_duck/video.log" 2>&1

