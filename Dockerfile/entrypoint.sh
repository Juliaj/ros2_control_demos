#!/bin/sh

. /opt/ros/"${ROS_DISTRO}"/setup.sh
cd /home/ros2_ws

# Build if source exists and install directory is missing or empty
if [ -d "/home/ros2_ws/src/ros2_control_demos" ] && [ ! -d "install" ] || [ -z "$(ls -A install 2>/dev/null)" ]; then
  echo "Building ros2_control_demos from mounted source..."
  colcon build --cmake-args -DBUILD_TESTING=OFF --symlink-install --packages-up-to ros2_control_demos
fi

. install/setup.sh
exec "$@"
