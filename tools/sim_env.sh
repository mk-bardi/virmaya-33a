#!/usr/bin/env bash
# Environment for Gazebo Harmonic + ArduPilot SITL. Sourced by the launcher and by ~/.bashrc (see install.sh).
# VIRMAYA_SIM = this repository. Its models come first on the Gazebo resource path.
export VIRMAYA_SIM="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export GZ_VERSION=harmonic
case ":${GZ_SIM_RESOURCE_PATH:-}:" in
  *":$VIRMAYA_SIM/models:"*) ;;
  *) export GZ_SIM_RESOURCE_PATH="$VIRMAYA_SIM/models:$VIRMAYA_SIM/worlds:$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}" ;;
esac
case ":${GZ_SIM_SYSTEM_PLUGIN_PATH:-}:" in
  *":$HOME/ardupilot_gazebo/build:"*) ;;
  *) export GZ_SIM_SYSTEM_PLUGIN_PATH="$HOME/ardupilot_gazebo/build${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}" ;;
esac
[ -f "$HOME/venv-ardupilot/bin/activate" ] && source "$HOME/venv-ardupilot/bin/activate"
case ":$PATH:" in *":$HOME/ardupilot/Tools/autotest:"*) ;; *) export PATH="$HOME/ardupilot/Tools/autotest:$PATH" ;; esac

# chase camera: chase [metres behind] [metres up] [model]   e.g. chase 15 4   |  chase 5 1.5 virmaya  |  chase off
chase() {
  if [ "$1" = "off" ]; then
    gz topic -t /gui/track -m gz.msgs.CameraTrack -p "track_mode: NONE"; return
  fi
  local back="${1:-8}" up="${2:-2.5}" model="${3:-}"
  if [ -z "$model" ]; then
    model=$(gz model --list 2>/dev/null | grep -oE "$(ls "$VIRMAYA_SIM/models" | paste -sd'|')" | head -1)
  fi
  gz topic -t /gui/track -m gz.msgs.CameraTrack \
    -p "track_mode: FOLLOW, follow_target: {name: \"${model:-virmaya}\"}, follow_offset: {x: -${back}, y: 0, z: ${up}}, follow_pgain: 0.5, track_pgain: 0.5"
}

alias virmaya="$VIRMAYA_SIM/tools/simlaunch.sh virmaya"
alias virmaya-showroom='gz sim "$VIRMAYA_SIM/worlds/showroom.sdf"'
alias simstop='pkill -f "gz sim"; pkill -f arduplane; pkill -f sim_vehicle.py; pkill -f mavproxy.py; true'
