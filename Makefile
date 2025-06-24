SHELL := /bin/bash
ROS_DISTRO := foxy 
# BASE_CLANG := --build-base build_clang --install-base install_clang
BUILD_ARGS := --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON


# .PHONY: build run_bumpgo run_gz run_bridge
.PHONY: build run open_teleop

build:
	colcon build $(BUILD_ARGS) &&\
	[[ -f "./build/compile_commands.json" ]] &&\
	[[ ! -f "./compile_commands.json" ]] &&\
	ln -s ./build/compile_commands.json compile_commands.json &&\
	echo "compile_commands.json linked"

run:
	ros2 launch hardware_controller run.launch.py

# run_bumpgo:
# 	ros2 run fsm_bumpgo_cpp bumpgo --ros-args -r \
# 		input_scan:=/lidar -r output_vel:=/cmd_vel \
# 		--log-level info
#
# run_gz:
# 	gz sim building_robot.sdf
#
# run_bridge:
# 	ros2 launch ros_gz_bridge ros_gz_bridge.launch.py bridge_name:=ros_gz_bridge \
# 		config_file:=bridge.yaml

open_teleop:
	ros2 run teleop_twist_keyboard teleop_twist_keyboard \
		--ros-args -r cmd_vel:=/input_key
#
#
# source-ros:
# 	@echo "Sourcing ROS underlay $(ROS_DISTRO) environment..."
# 	source /opt/ros/$(ROS_DISTRO)/setup.zsh && echo "Done."
#
# source-ws:
# 	@echo "Sourcing workspace overlay environment..."
# 	source ./install/setup.zsh && echo "Done."

