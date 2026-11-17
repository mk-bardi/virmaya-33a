# Setting up a new machine

Tested on Ubuntu 22.04 and 24.04 with an NVIDIA or Intel GPU. Plan about an hour, most of it compiling ArduPilot.

## 1. Basics

```bash
sudo apt update
sudo apt install -y git curl lsb-release gnupg python3-venv python3-pip
```

## 2. Gazebo Harmonic

```bash
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt update
sudo apt install -y gz-harmonic
gz sim --versions        # should print 8.x
```

## 3. ArduPilot SITL

```bash
cd ~
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git checkout Plane-4.5
git submodule update --init --recursive
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile

python3 -m venv ~/venv-ardupilot
source ~/venv-ardupilot/bin/activate
pip install pymavlink MAVProxy matplotlib numpy
./waf configure --board sitl
./waf plane
```

## 4. ardupilot_gazebo plugin

```bash
sudo apt install -y libgz-sim8-dev rapidjson-dev libopencv-dev \
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl
cd ~
git clone https://github.com/ArduPilot/ardupilot_gazebo
cd ardupilot_gazebo
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j4
```

## 5. This repo

```bash
git clone https://github.com/mk-bardi/virmaya-33a.git ~/virmaya-33a
cd ~/virmaya-33a
./install.sh
source ~/.bashrc
```

`install.sh` checks steps 2 to 4 and adds this line to `~/.bashrc`:

```bash
source "/home/<you>/virmaya-33a/tools/sim_env.sh"   # virmaya-33a
```

That file sets `VIRMAYA_SIM`, the Gazebo resource and plugin paths, activates the venv and defines `virmaya`, `virmaya-showroom`, `simstop` and `chase`. If you move the repo, run `./install.sh` again.

## 6. First flight check

```bash
virmaya --wipe
```

The first time, SITL asks for an accelerometer calibration. In the MAVProxy window type `accelcalsimple`, wait for it to finish, then fly. Later launches do not need `--wipe`.

## Optional: the world builder

`tools/make_sim_world.py` makes a world from satellite imagery and terrain. It needs:

```bash
pip install rasterio pillow requests
```

and either a Mapbox token (`export MAPBOX_TOKEN=pk...`), an Esri ArcGIS key, or your own GeoTIFFs. Worlds it builds go to `~/sim_worlds/<name>/` and `virmaya <name>` finds them there automatically.
