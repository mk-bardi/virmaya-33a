# SahelSense

Onboard vegetation indexing for the Virmaya-33A. It runs on the Orange Pi 5 companion computer, reads images from the SIYI A8 mini, and turns them into geotagged vegetation maps during the flight.

Main parts:

- **Camera nodes** (ROS 2) for the gimbal camera.
- **GLI vegetation index** (Green Leaf Index) computed per image, saved with geotagged file names.
- **YOLOv8n detection**, run on the RK3588S NPU (RKNN INT8), with Ultralytics and ONNX back ends for testing on a PC.
- **MAVLink link** to the autopilot for position, attitude and camera triggering.

The code is being added to this folder.
