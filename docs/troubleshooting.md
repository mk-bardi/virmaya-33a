# Troubleshooting

**The aircraft is not visible in Gazebo.**
Check that the environment is loaded: `echo $VIRMAYA_SIM` should print the repo path. If it is empty, run `source ~/.bashrc` or `./install.sh` again. Then check the model is found: `gz model --list` while the sim runs.

**SITL says "Waiting for heartbeat" forever, or "FTP: request timed out".**
Gazebo must be running and unpaused before SITL connects. Close everything with `simstop` and launch again. If the model was regenerated, use `--wipe`.

**"3D Accel calibration needed".**
After a `--wipe`. In MAVProxy type `accelcalsimple`, wait for "calibration done".

**Parameter changes in virmaya.parm have no effect.**
SITL keeps saved parameters in `eeprom.bin`, and those win over the defaults file. Launch once with `--wipe`.

**The chase camera is far away or does not follow.**
Run `chase` again after the model has spawned, or `chase 5 1.5` for closer. If another sim repo is also loaded in your shell, name the model: `chase 8 2.5 virmaya`. Both `follow_pgain` and `track_pgain` are set by the function; older scripts that only set one leave the camera behind.

**Gazebo crashes when clicking an entity.**
The Component Inspector panel crashes on some setups. The shipped worlds leave it out. If you add it back from the GUI menu and it crashes, remove it again.

**Very low real-time factor (RTF under 50 %).**
Usually a heavy world. Use `virmaya runway` to check the PC is fine. For generated worlds, use smaller textures (`--tex 1024`) and keep vegetation visual only.

**Mission does nothing, or flies off somewhere strange.**
The mission is for a different location. `wp list` and check the coordinates. The first item must be a VTOL takeoff.

**Mapbox returns 429 even with a new token.**
The account is blocked, not rate limited (header `x-rate-limit-limit: -1`). Use Esri imagery or your own GeoTIFFs with `make_sim_world.py`.

**Scripts fail with `bash\r: No such file or directory`.**
A file was saved with Windows line endings. `.gitattributes` prevents this on new commits; fix an existing file with `sed -i 's/\r$//' <file>`.
