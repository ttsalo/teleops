# teleops

Onboard software and operator side software for a teleoperated rover.

## Architecture

Operator Laptop:
- teleop_twist_joy (joystick → cmd_vel)
- rviz2 (visualization)
- rqt (debugging tools)

Rover Pi (ROS 2 nodes):
- motor_controller_node (cmd_vel → serial commands to AVR) ✓
- avr_interface_node (serial ← telemetry from AVR)
- camera_node (publishes /image)
- video_bridge_node (streams video to operator)

AVR (Arduino Nano, ATmega328P):
- Not running ROS (just receives serial commands)
- Motor control via 2x L298N H-bridges (3 motors per side) ✓
- Safety timeout (500ms) ✓
- Encoder reading (planned)

## Hardware

- **Raspberry Pi 5** — runs ROS 2, communicates with Nano over USB serial
- **Arduino Nano** — motor control firmware, 115200 baud serial
- **2x L298N** dual H-bridge motor drivers — one per side (left/right)
- **6 DC motors** — 3 per side, wired in parallel per L298N driver
- Pin assignments:
  - Left motor: ENA (PWM) = D9, IN1 = D8, IN2 = D7
  - Right motor: ENB (PWM) = D10, IN3 = D12, IN4 = D11

## Serial Protocol

Pi → Nano: `M <left> <right>\n` where left/right are integers in [-255, 255]

## Repository Structure

```
ros2_ws/src/teleops/       # ROS 2 package
  teleops/
    motor_controller_node.py   # cmd_vel subscriber → serial writer
  launch/
    motor_controller.launch.py
  config/
    params.yaml

firmware/                  # AVR firmware (non-ROS)
  motor_controller/
    motor_controller.ino       # Arduino Nano motor control firmware
```

## Building

### ROS 2 package
```bash
cd ros2_ws
colcon build --packages-select teleops
source install/setup.bash
ros2 launch teleops motor_controller.launch.py
```

### AVR firmware
```bash
arduino-cli compile --fqbn arduino:avr:nano firmware/motor_controller
arduino-cli upload --fqbn arduino:avr:nano -p /dev/ttyUSB0 firmware/motor_controller
```

