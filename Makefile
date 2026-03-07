SHELL := /bin/bash
.PHONY: rover operator

rover:
	source /opt/ros/jazzy/setup.bash && \
	cd ros2_ws && \
	colcon build --packages-select teleops && \
	source install/setup.bash && \
	ros2 launch teleops avr_interface.launch.py

operator:
	source /opt/ros/jazzy/setup.bash && \
	{ [ -f ~/ros2_jazzy/install/local_setup.bash ] && source ~/ros2_jazzy/install/local_setup.bash || true; } && \
	cd ros2_ws && \
	colcon build --packages-select teleops_operator && \
	source install/setup.bash && \
	ros2 launch teleops_operator operator.launch.py
