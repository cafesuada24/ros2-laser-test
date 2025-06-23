ROS_DISTRO=jazzy

echo "Sourcing ROS $ROS_DISTRO overlay environment..."
source /opt/ros/$ROS_DISTRO/setup.zsh && echo "Done."

echo "Sourcing workspace overlay environment..."
source ./install/setup.zsh && echo "Done."
