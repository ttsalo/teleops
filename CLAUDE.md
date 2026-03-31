# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Teleoperated rover system with three components:
- **Rover Pi** (`ros2_ws/src/teleops/`) — ROS 2 Jazzy package running on Raspberry Pi 5
- **Operator Laptop** (`ros2_ws/src/teleops_operator/`) — ROS 2 package with joystick control
- **AVR Firmware** (`firmware/motor_controller/`) — Arduino Nano (ATmega328P), no ROS

## Build Commands

### ROS 2 packages (from `ros2_ws/`)
```bash

```

### AVR firmware
```bash
arduino-cli compile --fqbn arduino:avr:nano firmware/motor_controller
arduino-cli upload --fqbn arduino:avr:nano -p /dev/ttyUSB0 firmware/motor_controller
```

## Building and running

### Rover Pi
```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
colcon build --packages-select teleops
source install/setup.bash
ros2 launch teleops avr_interface.launch.py   # motor control + telemetry

ros2 launch teleops camera.launch.py           # H.264 UDP video stream, separate shell
```

### Operator Laptop
```bash
# Operator laptop may need two ROS 2 sources if Jazzy was source-built:
source /opt/ros/jazzy/setup.bash
source ~/ros2_jazzy/install/local_setup.bash
colcon build --packages-select teleops_operator
source install/setup.bash
ros2 launch teleops_operator operator.launch.py

# Receive camera stream, separate shell
gst-launch-1.0 udpsrc port=5600 \
  caps='application/x-rtp,media=video,clock-rate=90000,encoding-name=H264,payload=96' \
  ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! autovideosink sync=false
```

## Architecture

### cmd_vel → Motor PWM (avr_interface_node.py)
Tank steering: `left = linear - angular`, `right = linear + angular`, scaled to [-255, 255]. The node has a 500ms safety timeout — if no `cmd_vel` is received, it sends `M 0 0`. When serial is unavailable, commands are logged at INFO level instead.

### Serial Protocol (Pi ↔ AVR)
- Pi → Nano: `M <left> <right>\n` (integers [-255, 255])
- Nano → Pi: `S <battery_mv> <enc_left> <enc_right>\n` (every 100ms)
- Nano → Pi: `READY\n` (on startup)

The AVR also enforces a 500ms command timeout independently — it stops motors if no `M` command arrives.

### Encoder tick direction (firmware)
Encoder ISRs increment/decrement based on the current motor direction variable (`leftDir`/`rightDir`), since the encoders are single-channel (no quadrature).

### Networking
Default ROS 2 multicast discovery fails over point-to-point IP tunnels. CycloneDDS with unicast peers solves this. Edit `ros2_ws/src/teleops_operator/config/cyclonedds.xml`, replace `ROVER_PI_IP` with the actual IP, and set:
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///path/to/cyclonedds.xml
```

### Camera streaming
`camera_node.py` uses `libcamerasrc` (Pi camera via libcamera) → `video/x-raw,format=NV12` → x264enc (zerolatency) → RTP → UDP. The `format=NV12` cap is required on Pi 5 (PiSP pipeline); without it libcamerasrc falls back to raw Bayer format and the pipeline errors. Target host/port configured via `params.yaml` (`receiver_host`, `receiver_port`).

### Configuration
All ROS 2 node parameters are in `ros2_ws/src/teleops/config/params.yaml` (rover) and `ros2_ws/src/teleops_operator/config/joy_params.yaml` (operator). The joystick requires holding `enable_button` (button 0) to publish `cmd_vel`.
