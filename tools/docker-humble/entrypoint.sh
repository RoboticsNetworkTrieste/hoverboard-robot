#!/usr/bin/env bash
# Runs the container command with the ROS2 environment sourced.
source /opt/hoverboard/ros_env.sh
exec "$@"
