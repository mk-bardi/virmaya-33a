#!/usr/bin/env bash
# simlaunch.sh: start Gazebo + ArduPilot SITL + dashboard for one vehicle in one world.
#
#   virmaya <world> [--wipe] [--name TESTNAME] [--no-dash]          (alias from tools/sim_env.sh)
#   tools/simlaunch.sh <vehicle> <world> [--wipe] [--name TESTNAME] [--no-dash]
#     vehicle : virmaya   (any folder in models/ that has a params/<vehicle>.parm)
#     world   : runway (default) | baylands | any world made by tools/make_sim_world.py (in ~/sim_worlds/<name>)
#     --wipe  : clear SITL's saved parameters (only when switching vehicles; redo accelcalsimple after)
#     --name  : label for the dashboard results (default <vehicle>_<world>)
#     --no-dash : don't open the dashboard
#   simstop   : close everything
#
# Opens three terminals: Gazebo, SITL (type flight commands here), dashboard.

set -u
usage() { sed -n '2,14p' "$0"; exit 1; }
[ $# -ge 1 ] || usage
VEH="$1"; shift
WORLD="runway"
if [ $# -gt 0 ] && [ "${1#--}" = "$1" ]; then WORLD="$1"; shift; fi
WIPE=""; NAME=""; DASH=1
while [ $# -gt 0 ]; do
  case "$1" in
    --wipe) WIPE="-w" ;;
    --name) NAME="$2"; shift ;;
    --no-dash) DASH=0 ;;
    *) echo "unknown option $1"; exit 1 ;;
  esac
  shift
done

ENV="$(cd "$(dirname "$0")" && pwd)/sim_env.sh"
# shellcheck source=/dev/null
source "$ENV"
MODEL="$VEH"; PARM_FILE="$VIRMAYA_SIM/params/${VEH}.parm"; SPAWN_Z=0.6
if [ ! -f "$VIRMAYA_SIM/models/$VEH/model.sdf" ] || [ ! -f "$PARM_FILE" ]; then
  echo "Unknown vehicle '$VEH'. Known: $(ls "$VIRMAYA_SIM/params" | sed 's/\.parm$//' | tr '\n' ' ')"; usage
fi

# A world is a folder with world.sdf (without the vehicle) and world.env (WORLD_NAME, LOC = home lat,lon,alt,yaw,
# optional SPAWN_DZ). Repo worlds first, then worlds you generated with make_sim_world.py.
WORLD_DIR=""
for d in "$VIRMAYA_SIM/worlds/$WORLD" "$HOME/sim_worlds/$WORLD"; do
  [ -f "$d/world.env" ] && { WORLD_DIR="$d"; break; }
done
if [ -z "$WORLD_DIR" ]; then
  echo "No world '$WORLD'. Known: $(cd "$VIRMAYA_SIM/worlds" && ls -d */ 2>/dev/null | tr -d / | tr '\n' ' ')$(ls ~/sim_worlds 2>/dev/null | tr '\n' ' ')"; usage
fi
SPAWN_DZ=0
# shellcheck source=/dev/null
source "$WORLD_DIR/world.env"
[ -d "$WORLD_DIR/models" ] && EXTRA_RES="$WORLD_DIR/models"
# Put the vehicle into a copy of the world file, so it exists from the first moment and every Gazebo
# window sees it (spawning it later races with heavy worlds still loading in the window).
SZ=$(awk -v a="$SPAWN_Z" -v b="${SPAWN_DZ:-0}" 'BEGIN{print a+b}')
LAUNCH_DIR="$HOME/.cache/virmaya_sim"; mkdir -p "$LAUNCH_DIR"
WORLD_FILE="$LAUNCH_DIR/${WORLD}_${MODEL}.sdf"
awk -v m="$MODEL" -v z="$SZ" '/<\/world>/{print "    <include>\n      <uri>model://" m "</uri>\n      <name>" m "</name>\n      <pose>0 0 " z " 0 0 0</pose>\n    </include>"} {print}' \
  "$WORLD_DIR/world.sdf" > "$WORLD_FILE"
EXTRA_RES="${EXTRA_RES:-}"
NAME="${NAME:-${VEH}_${WORLD}}"

if command -v gnome-terminal >/dev/null; then
  term() { gnome-terminal --title="$1" -- bash -c "$2; echo; echo '[$1 finished - press Enter to close]'; read"; }
else
  term() { x-terminal-emulator -T "$1" -e bash -c "$2; read"; }
fi

echo "==> stopping old simulations"
pkill -f "gz sim" 2>/dev/null; pkill -f arduplane 2>/dev/null; pkill -f sim_vehicle.py 2>/dev/null; pkill -f mavproxy.py 2>/dev/null
sleep 2

echo "==> Gazebo: $WORLD_FILE"
term "Gazebo" "source '$ENV'; ${EXTRA_RES:+export GZ_SIM_RESOURCE_PATH='$EXTRA_RES':\$GZ_SIM_RESOURCE_PATH;} cd '$WORLD_DIR'; gz sim -v4 -r '$WORLD_FILE'"

echo -n "==> waiting for the world to load"
for _ in $(seq 1 600); do        # up to 20 min: first launch of a Fuel world downloads it
  if gz service -l 2>/dev/null | grep -q "/world/${WORLD_NAME}/create"; then echo " ok"; break; fi
  echo -n "."; sleep 2
done

sleep 3
# chase camera: 8 m behind, 2.5 m up, stiff enough to keep up with a plane at cruise (Gazebo's default lags far behind)
for _ in 1 2 3; do
  gz topic -t /gui/track -m gz.msgs.CameraTrack \
    -p "track_mode: FOLLOW, follow_target: {name: \"${MODEL}\"}, follow_offset: {x: -8, y: 0, z: 2.5}, follow_pgain: 0.5, track_pgain: 0.5" 2>/dev/null
  sleep 1
done
echo "==> chase camera on $MODEL (change it any time with: chase <metres behind> <metres up>)"

echo "==> SITL: ArduPlane + $(basename "$PARM_FILE") at $LOC ${WIPE:+(wiping saved parameters)}"
term "SITL - type flight commands here" "source '$ENV'; cd ~; sim_vehicle.py -v ArduPlane --model JSON $WIPE --custom-location=$LOC --add-param-file='$PARM_FILE' --console --map"

if [ "$DASH" = 1 ]; then
  echo -n "==> waiting for SITL"
  for _ in $(seq 1 180); do
    if (exec 3<>/dev/tcp/127.0.0.1/5762) 2>/dev/null; then echo " ok"; break; fi
    echo -n "."; sleep 2
  done
  sleep 5
  term "Dashboard" "source '$ENV'; cd ~; python3 '$VIRMAYA_SIM/tools/sim_dashboard.py' --pid --name '$NAME' --title '${MODEL^^}' --connect tcp:127.0.0.1:5762"
fi

cat <<EOF

Ready. In the SITL terminal, once it says 'pre-arm good':
    mode qloiter
    rc 3 1000
    arm throttle        (wait for ARMED)
    rc 3 1700
Logs: ~/logs/*.BIN    Dashboard results: ~/sim_results/
Stop everything: simstop
EOF
