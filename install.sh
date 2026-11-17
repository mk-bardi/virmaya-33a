#!/usr/bin/env bash
# One-time setup on a machine that already has Gazebo Harmonic, ArduPilot SITL and the ardupilot_gazebo
# plugin (see docs/setup.md). Safe to run again.
#
#   ./install.sh
#
# It adds one line to ~/.bashrc that loads tools/sim_env.sh, which gives you:
#   virmaya [world]                  start Gazebo + SITL + dashboard (default world: runway)
#   virmaya-showroom                 look at the aircraft up close (no flight)
#   simstop                          close everything
#   chase [back] [up] | chase off    chase camera
set -e
ROOT="$(cd "$(dirname "$0")" && pwd)"
chmod +x "$ROOT/tools/"*.sh "$ROOT/tools/"*.py "$ROOT/install.sh"

echo "Repository: $ROOT"
ok=1
check() { if eval "$2" >/dev/null 2>&1; then echo "  [ok]      $1"; else echo "  [missing] $1  -> $3"; ok=0; fi; }
echo "Checking prerequisites:"
check "Gazebo Harmonic (gz sim 8)" "gz sim --versions | grep -q '^8'" "docs/setup.md, step 2"
check "ArduPilot (sim_vehicle.py)"  "[ -f \$HOME/ardupilot/Tools/autotest/sim_vehicle.py ]" "docs/setup.md, step 3"
check "ardupilot_gazebo plugin"      "[ -f \$HOME/ardupilot_gazebo/build/libArduPilotPlugin.so ]" "docs/setup.md, step 4"
check "Python venv ~/venv-ardupilot" "[ -f \$HOME/venv-ardupilot/bin/activate ]" "docs/setup.md, step 3"

LINE="source \"$ROOT/tools/sim_env.sh\"   # virmaya-33a"
if grep -q "# virmaya-33a" ~/.bashrc 2>/dev/null; then
  sed -i "\|# virmaya-33a|c\\$LINE" ~/.bashrc
  echo "Updated the virmaya-33a line in ~/.bashrc"
else
  printf '\n%s\n' "$LINE" >> ~/.bashrc
  echo "Added the virmaya-33a line to ~/.bashrc"
fi

echo
[ "$ok" = 1 ] && echo "Done. Open a new terminal (or: source ~/.bashrc), then:  virmaya --wipe" \
             || echo "Done, but install the missing items above before flying."
