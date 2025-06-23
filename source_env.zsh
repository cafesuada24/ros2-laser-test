ROS_DISTRO=foxy

echo "Sourcing ROS $ROS_DISTRO overlay environment..."
source /opt/ros/$ROS_DISTRO/setup.sh && echo "Done."

echo "Sourcing workspace overlay environment..."
source ./install/setup.sh && echo "Done."
