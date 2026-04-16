# teleops

Onboard software and operator side software for a teleoperated rover.

## Architecture

Operator Laptop:
- joy_node (reads joystick hardware → /joy)
- teleop_twist_joy (joy → cmd_vel_raw)
- cmd_vel_scaler (scales cmd_vel_raw by twist axis → cmd_vel)
- rviz2 (visualization)
- rqt (debugging tools)

Rover Pi (ROS 2 nodes):
- avr_interface_node (bidirectional serial with AVR: cmd_vel → motor commands, telemetry → ROS topics) ✓
- camera_node (GStreamer H.264 stream via UDP to operator) ✓
- metrics_node (Prometheus exporter: battery, encoders, cmd_vel → HTTP /metrics) ✓

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
 - Model JGA25-370, 30 RPM, 6V, 4 kg·cm torque, no-load 0.2 A, load 0.5 A
- **Performance** (wheel diameter 130 mm):
  - Top speed: 0.20 m/s (30 RPM × π × 0.13 m / 60)
  - Drive force: 36.2 N (6 motors × 4 kg·cm / 6.5 cm radius × 9.81 m/s²)
- **Battery**: 12× Panasonic NCR18650B, 3P4S configuration
  - 14.4 V nominal, 16.8 V fully charged, 10.0 V cutoff
  - 10,200 mAh (10.2 Ah), ~147 Wh
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
| `/cmd_vel_raw` | `geometry_msgs/Twist` | Unscaled velocity from teleop_twist_joy |
| `/cmd_vel` | `geometry_msgs/Twist` | Speed-scaled velocity commands to rover |
| `/battery_voltage` | `std_msgs/Float32` | Battery voltage in volts |
| `/encoder_ticks` | `std_msgs/Int32MultiArray` | Cumulative encoder ticks [left, right] |

## Prometheus Metrics

`metrics_node` exposes a Prometheus-compatible `/metrics` endpoint on port 9101 (configurable via `params.yaml`).

| Metric | Type | Description |
|--------|------|-------------|
| `teleops_battery_voltage_volts` | Gauge | Battery voltage in volts |
| `teleops_encoder_ticks_total{side}` | Counter | Cumulative encoder ticks, `side` ∈ {left, right} |
| `teleops_cmd_vel_linear_mps` | Gauge | Current `cmd_vel` linear velocity (m/s) |
| `teleops_cmd_vel_angular_radps` | Gauge | Current `cmd_vel` angular velocity (rad/s) |
| `teleops_telemetry_age_seconds` | Gauge | Seconds since last AVR telemetry received |
| `teleops_avr_ready` | Gauge | 1 if AVR telemetry is current, 0 if stale |

Add to your Prometheus `scrape_configs`:
```yaml
- job_name: teleops_rover
  static_configs:
    - targets: ['<pi-ip>:9101']
```

## Repository Structure

```
ros2_ws/src/teleops/            # Rover Pi ROS 2 package
  teleops/
    avr_interface_node.py           # Bidirectional serial: cmd_vel + telemetry
    camera_node.py                  # GStreamer H.264 UDP streaming
    metrics_node.py                 # Prometheus metrics exporter
  launch/
    avr_interface.launch.py
    camera.launch.py
    metrics.launch.py
  config/
    params.yaml

ros2_ws/src/teleops_operator/   # Operator laptop ROS 2 package
  config/
    joy_params.yaml                 # Joystick axis/button mapping
    cyclonedds.xml                  # Unicast peer discovery config
  teleops_operator/
    cmd_vel_scaler.py               # Scales cmd_vel by joystick twist axis
  launch/
    operator.launch.py              # Launches joy_node, teleop_twist_joy, cmd_vel_scaler

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
sudo apt install python3-prometheus-client
sudo apt install python3-gi python3-gi-cairo \
  gir1.2-gstreamer-1.0 gir1.2-gst-plugins-base-1.0 \
  gstreamer1.0-tools gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly \
  gstreamer1.0-libav libcamera-dev gstreamer1.0-libcamera
```

#### Raspberry Pi 5: libcamera IPA

Ubuntu's `libcamera-ipa` only includes the Pi 4 (VC4) IPA. Pi 5 needs the PiSP IPA
(`ipa_rpi_pisp.so`) from the Raspberry Pi Foundation's archive:

```bash
curl -fsSL https://archive.raspberrypi.com/debian/raspberrypi.gpg.key \
  | sudo gpg --dearmor -o /etc/apt/trusted.gpg.d/raspberrypi.gpg
echo "deb http://archive.raspberrypi.com/debian/ bookworm main" \
  | sudo tee /etc/apt/sources.list.d/raspi.list
sudo apt update && sudo apt install libcamera-ipa libcamera0.2
```

> Note: this mixes a Debian bookworm repo into Ubuntu. Only `libcamera-ipa`
> and `libcamera0.2` are needed from it; `libcamera-tools` has an Ubuntu-incompatible
> dependency (`libjpeg62-turbo`) and should not be installed from this repo.

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

# In a separate terminal — Prometheus metrics on :9101/metrics:
ros2 launch teleops metrics.launch.py
```

Before launching the camera node, set the operator laptop's IP in
`ros2_ws/src/teleops/config/params.yaml`:
```yaml
camera_node:
  ros__parameters:
    receiver_host: "192.168.x.x"  # operator laptop IP
```

### Operator laptop — ROS 2 package
```bash
source /opt/ros/jazzy/setup.bash  # apt-installed packages (joy, teleop_twist_joy)
source ~/ros2_jazzy/install/local_setup.bash  # source-built ROS 2
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

## Deployment

One-time setup steps for the rover Pi and operator laptop. Run these after
cloning the repo.

### Rover Pi

**SSH server keepalives** — prevents connections from hanging on mobile/tunnel links:
```bash
make deploy-ssh-server
```
Installs `config/ssh/sshd.conf` to `/etc/ssh/sshd_config.d/teleops.conf` and
reloads sshd.

**Systemd services** — runs avr_interface, camera, and metrics as system
services that start on boot and restart on failure, independent of SSH sessions:
```bash
make deploy-systemd
sudo systemctl start teleops-avr teleops-camera teleops-metrics
```

To stop and remove:
```bash
make undeploy-systemd
```

Check service status:
```bash
systemctl status teleops-avr teleops-camera teleops-metrics
journalctl -u teleops-avr -f   # live log
```

> **Note:** `deploy-systemd` installs the last-built binaries. Run
> `colcon build --packages-select teleops` before deploying after any code change.

### Operator laptop

**SSH client keepalives** — add to `~/.ssh/config`:
```bash
make deploy-ssh-client
```
Prints the `config/ssh/client.conf` snippet to paste. Replace `<ROVER_IP>` with
the rover's actual IP (WireGuard, tunnel, etc.).

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
