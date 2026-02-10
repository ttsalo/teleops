# teleops

Onboard software and operator side software for a teleoperated rover.

## Architecture

Operator Laptop:
- teleop_twist_joy (joystick → cmd_vel)
- rviz2 (visualization)
- rqt (debugging tools)

Rover Pi (ROS 2 nodes):
- avr_interface_node (bidirectional serial with AVR: cmd_vel → motor commands, telemetry → ROS topics) ✓
- camera_node (publishes /image)
- video_bridge_node (streams video to operator)

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
- **2x L298N** dual H-bridge motor drivers — one per side (left/right)
- **6 DC motors** — 3 per side, wired in parallel per L298N driver
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
ros2_ws/src/teleops/       # ROS 2 package
  teleops/
    avr_interface_node.py      # Bidirectional serial: cmd_vel + telemetry
  launch/
    avr_interface.launch.py
  config/
    params.yaml

firmware/                  # AVR firmware (non-ROS)
  motor_controller/
    motor_controller.ino       # Arduino Nano motor control + telemetry firmware
```

## Building

### ROS 2 package
```bash
cd ros2_ws
colcon build --packages-select teleops
source install/setup.bash
ros2 launch teleops avr_interface.launch.py
```

### AVR firmware
```bash
arduino-cli compile --fqbn arduino:avr:nano firmware/motor_controller
arduino-cli upload --fqbn arduino:avr:nano -p /dev/ttyUSB0 firmware/motor_controller
```
