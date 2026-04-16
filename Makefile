SHELL := /bin/bash
.PHONY: rover camera metrics operator \
        deploy-ssh-server deploy-ssh-client \
        deploy-systemd undeploy-systemd

rover:
	source /opt/ros/jazzy/setup.bash && \
	cd ros2_ws && \
	colcon build --packages-select teleops && \
	source install/setup.bash && \
	ros2 launch teleops avr_interface.launch.py

camera:
	source /opt/ros/jazzy/setup.bash && \
	cd ros2_ws && \
	source install/setup.bash && \
	ros2 launch teleops camera.launch.py

metrics:
	source /opt/ros/jazzy/setup.bash && \
	cd ros2_ws && \
	source install/setup.bash && \
	ros2 launch teleops metrics.launch.py

operator:
	source /opt/ros/jazzy/setup.bash && \
	{ [ -f ~/ros2_jazzy/install/local_setup.bash ] && source ~/ros2_jazzy/install/local_setup.bash || true; } && \
	cd ros2_ws && \
	colcon build --packages-select teleops_operator && \
	source install/setup.bash && \
	ros2 launch teleops_operator operator.launch.py

deploy-ssh-server:
	sudo install -m 644 config/ssh/sshd.conf /etc/ssh/sshd_config.d/teleops.conf
	sudo sshd -t
	sudo systemctl reload ssh

deploy-ssh-client:
	@echo "=== Paste into ~/.ssh/config on this laptop ==="
	@cat config/ssh/client.conf

deploy-systemd:
	sudo install -m 755 config/systemd/teleops-avr.sh     /usr/local/bin/teleops-avr.sh
	sudo install -m 755 config/systemd/teleops-camera.sh  /usr/local/bin/teleops-camera.sh
	sudo install -m 755 config/systemd/teleops-metrics.sh /usr/local/bin/teleops-metrics.sh
	sudo install -m 644 config/systemd/teleops-avr.service     /etc/systemd/system/
	sudo install -m 644 config/systemd/teleops-camera.service  /etc/systemd/system/
	sudo install -m 644 config/systemd/teleops-metrics.service /etc/systemd/system/
	sudo systemctl daemon-reload
	sudo systemctl enable teleops-avr teleops-camera teleops-metrics
	@echo "Run: sudo systemctl start teleops-avr teleops-camera teleops-metrics"

undeploy-systemd:
	-sudo systemctl stop    teleops-avr teleops-camera teleops-metrics
	-sudo systemctl disable teleops-avr teleops-camera teleops-metrics
	sudo rm -f /etc/systemd/system/teleops-avr.service \
	           /etc/systemd/system/teleops-camera.service \
	           /etc/systemd/system/teleops-metrics.service \
	           /usr/local/bin/teleops-avr.sh \
	           /usr/local/bin/teleops-camera.sh \
	           /usr/local/bin/teleops-metrics.sh
	sudo systemctl daemon-reload
