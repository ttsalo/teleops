# teleops

Onboard software and operator side software for a teleoperated rover.

## Architecture

Operator Laptop:
- teleop_twist_joy (joystick → cmd_vel)
- rviz2 (visualization)
- rqt (debugging tools)

Rover Pi (ROS 2 nodes):
- avr_interface_node (bidirectional serial with AVR: cmd_vel → motor commands, telemetry → ROS topics) ✓
- camera_node (GStreamer H.264 stream via UDP to operator) ✓

AVR (Arduino Nano, ATmega328P):
- Not running ROS (bidirectional serial with Pi)
- Motor control via 2x L298N H-bridges (3 motors per side) ✓
- Safety timeout (500ms) ✓
- Encoder reading via interrupts on D2/D3 ✓
- Battery voltage monitoring via A0 voltage divider ✓
- Telemetry output every 100ms ✓

## Hardware

- **Raspberry Pi 5** — runs ROS 2, communicates with Nano over USB serial
- **Arduino Nano** — motor control firmware, 115200 baud serial
- **1x L298N** dual H-bridge motor driver — channel A for left, channel B for right
- **6 DC motors** — 3 per side, wired in parallel per channel
 - Model JGA25-370, 30 RPM, 6V, no-load 0.2 A, load 0.5 A
- Pin assignments:
  - Left motor: ENA (PWM) = D9, IN1 = D8, IN2 = D7
  - Right motor: ENB (PWM) = D10, IN3 = D12, IN4 = D11
  - Left encoder: D2 (INT0)
  - Right encoder: D3 (INT1)
  - Battery voltage: A0 (via divider, R1=10k / R2=3.3k)

## Serial Protocol (bidirectional)

- **Pi → Nano**: `M <left> <right>\n` where left/right are integers in [-255, 255]
- **Nano → Pi**: `S <battery_mv> <enc_left> <enc_right>\n` (every 100ms)
- **Nano → Pi**: `READY\n` (on startup)

## ROS Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Motor velocity commands (subscribed) |
| `/battery_voltage` | `std_msgs/Float32` | Battery voltage in volts |
| `/encoder_ticks` | `std_msgs/Int32MultiArray` | Cumulative encoder ticks [left, right] |

## Repository Structure

```
ros2_ws/src/teleops/            # Rover Pi ROS 2 package
  teleops/
    avr_interface_node.py           # Bidirectional serial: cmd_vel + telemetry
    camera_node.py                  # GStreamer H.264 UDP streaming
  launch/
    avr_interface.launch.py
    camera.launch.py
  config/
    params.yaml

ros2_ws/src/teleops_operator/   # Operator laptop ROS 2 package
  config/
    joy_params.yaml                 # Joystick axis/button mapping
    cyclonedds.xml                  # Unicast peer discovery config
  launch/
    operator.launch.py              # Launches joy_node + teleop_twist_joy_node

firmware/                       # AVR firmware (non-ROS)
  motor_controller/
    motor_controller.ino            # Arduino Nano motor control + telemetry firmware
```

## Install prerequisites

### ROS 2 (both sides, Ubuntu 24.04)

Install ROS 2 Jazzy following the [official instructions](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html):
```bash
# Add ROS 2 apt repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update

# Install ROS 2 base and build tools
sudo apt install ros-jazzy-ros-base python3-colcon-common-extensions
```

### Rover Pi additional dependencies
```bash
pip install pyserial
sudo apt install python3-gi gstreamer1.0-tools gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly \
  gstreamer1.0-libav libcamera-dev gstreamer1.0-libcamera
```

### Operator laptop GStreamer (for receiving video)
```bash
sudo apt install gstreamer1.0-tools gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-libav
```

### Operator laptop additional dependencies
```bash
sudo apt install ros-jazzy-joy ros-jazzy-teleop-twist-joy ros-jazzy-rmw-cyclonedds-cpp
```

### Arduino CLI (for firmware builds)
```bash
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
sudo mv bin/arduino-cli /usr/local/bin/

# Install Arduino AVR core
arduino-cli core update-index
arduino-cli core install arduino:avr
```

## Building

### Rover Pi — ROS 2 package
```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
colcon build --packages-select teleops
source install/setup.bash
ros2 launch teleops avr_interface.launch.py

# In a separate terminal — camera streaming:
ros2 launch teleops camera.launch.py
```

### Operator laptop — ROS 2 package
```bash
# source /opt/ros/jazzy/setup.bash  # if installed from packages
source ~/ros2_jazzy/install/local_setup.bash
cd ros2_ws
colcon build --packages-select teleops_operator
source install/setup.bash
ros2 launch teleops_operator operator.launch.py

# In a separate terminal — receive camera stream:
gst-launch-1.0 udpsrc port=5600 \
  caps='application/x-rtp,media=video,clock-rate=90000,encoding-name=H264,payload=96' \
  ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! autovideosink sync=false
```

### AVR firmware
```bash
arduino-cli compile --fqbn arduino:avr:nano firmware/motor_controller
arduino-cli upload --fqbn arduino:avr:nano -p /dev/ttyUSB0 firmware/motor_controller
```

## Networking with CycloneDDS

By default ROS 2 uses multicast for node discovery, which doesn't work over
point-to-point IP tunnels (mobile data, VPN, etc.). CycloneDDS with unicast
peer discovery solves this.

### Setup (both sides)

1. Install CycloneDDS RMW (if not already):
   ```bash
   sudo apt install ros-jazzy-rmw-cyclonedds-cpp
   ```

2. Edit `ros2_ws/src/teleops_operator/config/cyclonedds.xml` — replace
   `ROVER_PI_IP` with the rover Pi's IP address.

3. On the rover Pi, create a similar `cyclonedds.xml` with the operator
   laptop's IP as the peer address.

4. Set environment variables before launching nodes (add to your shell rc):
   ```bash
   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
   export CYCLONEDDS_URI=file:///path/to/cyclonedds.xml
   ```

Both sides need `RMW_IMPLEMENTATION` and `CYCLONEDDS_URI` set. The IP tunnel
itself (WireGuard, SSH tunnel, etc.) is set up separately by the user.
