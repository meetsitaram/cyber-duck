#!/bin/bash
# The whole chain for one duck, from whichever worktree this is in. Takes its turn at Fusion and at rendering if TEAM_DIR is set.
#
#   MILLI_RECIPE=hd1910 AGENT=hd1910 bash tools/pipeline.sh gates     offline checks, docs, prints from the spec. Free: run it as often as you like
#   ... pipeline.sh fusion    linked Fusion build, prints from Fusion, audit: 10 min the first time, then 1-3 min, as it rebuilds only the links that changed
#   ... pipeline.sh final     the one-document Fusion build, for the repo's .f3d (about 12 min). Milestones only
#   ... pipeline.sh reveal    the one-document build and the 30-second reveal's frames, in one turn (about 40 min)
#   ... pipeline.sh step      the STEP, from the document final or reveal saved in the hub, in a turn of its own (a minute or two)
#   ... pipeline.sh video    the 5-minute build and the guide's clips (about 10 min; needs Fusion's bodies from `fusion` or `reveal`)
#
# Stops at the first failing gate. Reads the whole of each check's output, as a failure isn't always on its last line.
RECIPE=${MILLI_RECIPE:?set MILLI_RECIPE}
AGENT=${AGENT:-$RECIPE}
cd "$(dirname "$0")/.."
export PYTHONIOENCODING=utf-8 MILLI_RECIPE=$RECIPE
PY=${PY:-/c/Users/meets/.venvs/duck/Scripts/python.exe}
TEAM=${TEAM:-$HOME/.claude/skills/agentic-team/scripts/team.py}
D=docs/images/assembly/$RECIPE
WORK=${TEMP:-/tmp}/pipeline_$RECIPE
mkdir -p "$WORK"
HELD=

take() {   # resource, minutes, why: waits its turn, if there's a team
  [ -n "$TEAM_DIR" ] || return 0
  while true; do
    "$PY" "$TEAM" wait "$1" --dir "$TEAM_DIR" --agent "$AGENT" --minutes "$2" --why "$3" --priority "${PRIORITY:-1}" --timeout 300
    rc=$?
    [ $rc -eq 0 ] && { HELD=$1; return 0; }
    [ $rc -eq 5 ] && { echo "$1 is on hold: stopping here"; exit 5; }
    [ $rc -eq 3 ] || { echo "scheduler error $rc"; exit 1; }
  done
}
turn_end() {   # before Fusion goes to the next duck, even after a failure: saved, closed, and nothing left open. If that can't be done, nobody else gets a turn on it
  left=$(timeout 240 "$PY" -I tools/fusion_call.py cad/fusion/turn_end.py --write 2>&1 | tail -3 | cut -c1-240)
  echo "[turn end] $left"
  echo "$left" | grep -q "open now 0" || "$PY" "$TEAM" hold fusion --dir "$TEAM_DIR" --why "$AGENT's turn end left Fusion unclosed: $(echo "$left" | tail -1 | cut -c1-120)"
}
give() {
  h=$HELD
  HELD=
  [ -n "$TEAM_DIR" ] && [ -n "$h" ] || return 0
  # only while the turn is still ours: 2026-11-05, a turn that had expired closed the next duck's build
  if [ "$h" = fusion ]; then
    if "$PY" "$TEAM" renew fusion --dir "$TEAM_DIR" --agent "$AGENT" --minutes 5 >/dev/null 2>&1; then turn_end; else echo "[turn end] skipped: the turn had already ended"; return 0; fi
  fi
  "$PY" "$TEAM" release "$h" --dir "$TEAM_DIR" --agent "$AGENT"
}
trap give EXIT

gate() {   # name, command
  n=$1; shift
  s=$(date +%s)
  out=$("$@" 2>&1); rc=$?
  last=$(echo "$out" | tail -1 | cut -c1-150)
  echo "[$n] rc=$rc $(( $(date +%s) - s )) s: $last"
  if [ $rc -ne 0 ] || echo "$out" | grep -qE "(^|[ ,])[1-9][0-9]* failing"; then
    echo "$out" | grep -iE "fail|error|Traceback" | head -25 | cut -c1-220
    echo "STOPPED at $n"
    exit 1
  fi
}

gates() {
  if [ "$RECIPE" = big ]; then      # its own spec, and no printed parts: buildable, export, printable and beak don't apply to it. Its assembly has its own checker
    gate spec "$PY" -I spec/big_duck.py
    gate checks "$PY" -I tools/checks.py
    gate assembly "$PY" -I tools/big_assembly.py
    gate docs "$PY" -I tools/make_docs.py
    gate mass "$PY" -I tools/mass_model.py
    return
  fi
  gate spec "$PY" -I spec/robot.py
  gate checks "$PY" -I tools/checks.py
  gate assembly "$PY" -I tools/assembly.py
  gate buildable "$PY" -I tools/buildable.py
  gate docs "$PY" -I tools/make_docs.py
  gate export "$PY" -I tools/export_prints.py
  gate printable "$PY" -I tools/printable.py
  gate beak "$PY" -I tools/beak_linkage.py
  gate mass "$PY" -I tools/mass_model.py
  if [ "$RECIPE" = hybrid ]; then gate wires "$PY" -I tools/wire_routes.py; fi
  # (export writes prints/ from the spec's shapes, over any from Fusion: 2026-11-05, this nearly got committed)
  echo "NOTE: prints/ now hold the spec's unrounded shapes. Before committing, run pipeline.sh fusion, or restore them: git restore prints"
}

keep_turn() {
  [ -n "$TEAM_DIR" ] || return 0
  while sleep 60; do
    "$PY" "$TEAM" renew fusion --dir "$TEAM_DIR" --agent "$AGENT" --minutes 5 >/dev/null 2>&1 || return 0
  done
}
fusion_run() {   # script, lines to show. A Fusion script that fails stops the chain: it used to go on, and print DONE over a stale clash report
  keep_turn &                                      # a lease mustn't run out under a call that's still open: the next duck's script runs inside it, and both fail
  keeper=$!
  out=$("$PY" -I tools/fusion_call.py "$1" --write --recipe "$RECIPE" 2>&1)
  kill $keeper 2>/dev/null
  echo "$out" | grep -v "bodies, " | tail -"$2" | cut -c1-240
  if echo "$out" | grep -qE "Traceback|RPC error"; then
    echo "STOPPED: the Fusion script failed"
    exit 1
  fi
}

build() {
  gate verify_write "$PY" -I tools/verify_fusion.py write
  sed 's/^KEEP_OPEN = False/KEEP_OPEN = True/' cad/fusion/full_build.py > "$WORK/fb_open.py"
  s=$(date +%s)
  fusion_run "$WORK/fb_open.py" 14
  echo "[fusion build] $(( $(date +%s) - s )) s"
  gate prints "$PY" -I tools/prints_from_fusion.py
  gate audit "$PY" -I tools/audit_prints.py
  gate compare "$PY" -I tools/verify_fusion.py compare
}

linked() {
  if [ "$RECIPE" = big ]; then      # BIG_ONLY="'head'," BIG_SECONDS=600: a trial, or a turn with a time limit. It has no printed parts, so no prints, audit or compare
    sed -e "s/^BIG_ONLY = None/BIG_ONLY = ${BIG_ONLY:-None}/" -e "s/^BIG_SECONDS = 900/BIG_SECONDS = ${BIG_SECONDS:-900}/" -e "s/^BIG_ASSEMBLY = True/BIG_ASSEMBLY = ${BIG_ASSEMBLY:-False}/" -e "s/^BIG_CLASH = True/BIG_CLASH = ${BIG_CLASH:-True}/" cad/fusion/linked_build.py > "$WORK/big.py"
    s=$(date +%s)
    fusion_run "$WORK/big.py" 30
    echo "[linked build] $(( $(date +%s) - s )) s"
    return
  fi
  gate verify_write "$PY" -I tools/verify_fusion.py write
  s=$(date +%s)
  fusion_run cad/fusion/linked_build.py 12
  echo "[linked build] $(( $(date +%s) - s )) s"
  gate prints "$PY" -I tools/prints_from_fusion.py
  gate audit "$PY" -I tools/audit_prints.py
  gate compare "$PY" -I tools/verify_fusion.py compare
}

case $1 in
  gates)
    gates
    ;;
  fusion)
    gates
    take fusion "${FUSION_MIN:-5}" "$RECIPE linked build"
    linked
    ;;
  final)
    gates
    take fusion 15 "$RECIPE one-document build"
    build
    ;;
  reveal)
    gates
    if [ "$RECIPE" = big ]; then      # offline, in MuJoCo from the spec's shapes: no Fusion turn (tools/render_big_video.py)
      take render 10 "$RECIPE reveal, offline"
      "$PY" tools/render_big_video.py reveal 2>&1 | tail -3 | cut -c1-150
      give
    else
    take fusion 40 "$RECIPE build and reveal frames"
    build
    for a in 0 150 300 450 600 750; do
      b=$((a + 149))
      [ -n "$TEAM_DIR" ] && "$PY" "$TEAM" renew fusion --dir "$TEAM_DIR" --agent "$AGENT" --minutes 20
      sed "s/^FIRST, LAST = 0, 2/FIRST, LAST = $a, $b/" cad/fusion/reveal_frames.py > "$WORK/rv.py"
      "$PY" -I tools/fusion_call.py "$WORK/rv.py" --write 2>&1 | tail -1 | cut -c1-120
    done
    "$PY" -I tools/reveal_video.py 2>&1 | tail -1 | cut -c1-140      # the frames' folder is shared: join them before giving Fusion up
    give
    fi
    FF=$("$PY" -c "import imageio_ffmpeg as i;print(i.get_ffmpeg_exe())")
    "$FF" -y -loglevel error -i "$D/reveal_30s.mp4" -c:v libx264 -preset slow -crf 20 -pix_fmt yuv420p -movflags +faststart "$WORK/reveal.mp4" && cp "$WORK/reveal.mp4" "$D/reveal_30s.mp4"
    "$FF" -y -loglevel error -i "$D/reveal_30s.mp4" -vf "fps=12,scale=360:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=128[p];[b][p]paletteuse=dither=bayer:bayer_scale=4" "$D/reveal.gif"
    ls -la "$D/reveal_30s.mp4" "$D/reveal.gif"
    ;;
  step)
    # after final or reveal, in its own turn: straight after a first save Fusion won't reopen the document, and a STEP of it has no solids
    take fusion 5 "$RECIPE STEP export"
    fusion_run cad/fusion/export_step.py 4
    n=$(grep -c "MANIFOLD_SOLID_BREP" "cad/duck-droids-$RECIPE.step")
    echo "[step] cad/duck-droids-$RECIPE.step: $(wc -c < "cad/duck-droids-$RECIPE.step") bytes, $n solids"
    [ "$n" -gt 0 ] || { echo "STOPPED: the STEP has no solids"; exit 1; }
    ;;
  video)
    if [ "$RECIPE" = big ]; then      # big_assembly's order, offline in MuJoCo; no printed parts, so no Fusion bodies, floaters or parts layout
      gate assembly "$PY" -I tools/big_assembly.py
      take render 45 "$RECIPE 5-minute build and clips"
      "$PY" tools/render_big_video.py build 2>&1 | tail -6 | cut -c1-200
      "$PY" tools/cut_clips.py 2>&1 | tail -3 | cut -c1-110
    else
    # the film uses Fusion's rounded bodies, and without them falls back on the spec's sharp shapes without a word
    [ -d "${TEMP:-/tmp}/duck_fusion_stl/$RECIPE" ] || { echo "no Fusion bodies for $RECIPE: run pipeline.sh fusion first"; exit 1; }
    take render 10 "$RECIPE 5-minute build and clips"
    gate floaters "$PY" tools/video_floaters.py
    "$PY" tools/render_pov_video.py 2>&1 | grep -v "^dry" | tail -3 | cut -c1-150
    "$PY" tools/cut_clips.py 2>&1 | tail -3 | cut -c1-110
    gate parts_layout "$PY" tools/render_parts_layout.py
    fi
    FF=$("$PY" -c "import imageio_ffmpeg as i;print(i.get_ffmpeg_exe())")
    "$FF" -y -loglevel error -i "$D/build_5min.mp4" -c:v libx264 -preset medium -crf 24 -pix_fmt yuv420p -movflags +faststart "$WORK/build.mp4" && cp "$WORK/build.mp4" "$D/build_5min.mp4"
    "$PY" -I tools/make_docs.py | tail -1 | cut -c1-70
    ls -la "$D/build_5min.mp4"
    ;;
  *)
    echo "gates, fusion, final, reveal, step or video"
    exit 2
    ;;
esac
echo "PIPELINE $1 DONE"
