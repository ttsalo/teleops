# teleops

Onboard software and operator side software for a teleoperated rover.

Operator Laptop:
- teleop_twist_joy (joystick → cmd_vel)
- rviz2 (visualization)
- rqt (debugging tools)

Rover Pi (ROS 2 nodes):
- motor_controller_node (cmd_vel → serial commands to AVR)
- avr_interface_node (serial ← telemetry from AVR)
- camera_node (publishes /image)
- video_bridge_node (streams video to operator)

AVR:
- Not running ROS (just receives serial commands)
- Motor control, encoder reading, safety timeout

