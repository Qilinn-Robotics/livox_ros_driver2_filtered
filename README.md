# Livox MID360 Filtered Driver

This fork is trimmed for the real Ranger Mini V3 setup:

- Ubuntu 22.04
- ROS 2 Humble
- Livox MID360
- `/livox/lidar` published as Livox `CustomMsg`
- Embedded self-filter before publishing `/livox/lidar`

FAST-LIO can keep:

```yaml
lidar_topic: /livox/lidar
```

## Build

Install Livox-SDK2 first, then:

```bash
cd ~/CHAMP_ws/livox_ros_driver2_filtered
rm -rf build install log
./build.sh
```

## Run

```bash
cd ~/CHAMP_ws/livox_ros_driver2_filtered
./start_mid360_filtered_driver.sh
```

The script isolates this workspace from previously sourced overlays and checks
that `livox_ros_driver2` resolves to this repository's `install/` directory.

## Output

- `/livox/lidar`: filtered Livox `CustomMsg`
- `/livox/imu`: Livox IMU

The embedded filter uses the real Ranger Mini V3 body and IMPS boxes configured
in `launch_ROS2/msg_MID360_launch.py`.
