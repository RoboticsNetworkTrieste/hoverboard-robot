# ROS2 Humble container

The ROS2 side of the project targets Humble. This container provides it on any Linux host with Docker.
To run the robot, use the scripts in [`scripts/`](../../scripts/), which start this container by themselves: see
[`ros2/README.md`](../../ros2/README.md). Open a container directly for debugging or calibration.

```
tools/docker-humble/run.sh --build     # first time, or after editing the Dockerfile
tools/docker-humble/run.sh             # interactive shell
tools/docker-humble/run.sh <command>   # one command, e.g. ros2 topic list
```

- `ros2/` from this repo is mounted at `/ws/src`; build with `colcon build --symlink-install` from `/ws`.
  Build outputs live in Docker volumes, not in the repo. `docker compose down -v` (from this folder) wipes them.
- Host networking: the container reaches the ESP at the same addresses as the host
  (192.168.7.1 over USB, 192.168.4.1 on the ESP access point, the static IP from menuconfig in
  station mode). `hoverboard.local` does not resolve inside the container: use the IP.
- `ldlidar_stl_ros2` is prebuilt in the image; Nav2, `slam_toolbox`, `robot_localization` and
  `teleop_twist_keyboard` are installed from the ROS2 apt repository.
- GUI tools (`rviz2`): run `xhost +local:` on the host once per login session.
- Every shell opened with `run.sh` is a new container; several can run at the same time and see each other's topics.

## Link check by manual driving

Quick check of the bridge and the LIDAR, without EKF, SLAM or Nav2.

```
tools/docker-humble/teleop_debug.sh [esp_host]   # default 192.168.7.1 (USB)
```

Builds the workspace, then starts the bridge, the LIDAR driver and RViz in the background and keyboard teleop in the
terminal. Keep the keyboard focus on the terminal, not on RViz. Ctrl-C closes everything; logs are in
`/ws/log/teleop_debug/`.
