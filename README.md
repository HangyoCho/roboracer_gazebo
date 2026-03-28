# Roboracer Gazebo Simulation

Gazebo simulation package for a 1/10 scale autonomous racing vehicle equipped with a Livox Mid-360 LiDAR.

## Features

- **Ackermann Steering** — Realistic steering kinematics with inner/outer wheel differentiation
- **Livox Mid-360 LiDAR** — Non-repetitive scan pattern, PointCloud2 output (10Hz, 20k pts/frame)
- **IMU** — 200Hz built-in IMU sensor on the LiDAR frame
- **Ground Truth Publisher** — Broadcasts `map -> base_link` TF, publishes `base_pose` and `base_odom`
- **Joystick Support** — Deadman switch + analog stick control via joy_node
- **ICRA Racetrack** — Pre-built racing course world
- **Multi-Agent** — Supports spawning two vehicles simultaneously

## Prerequisites

- Ubuntu 20.04
- ROS Noetic
- Gazebo Classic 11

## Installation

```bash
mkdir -p ~/ws_gazebo_roboracer/src && cd ~/ws_gazebo_roboracer/src
git clone git@github.com:HangyoCho/roboracer_gazebo.git unicorn_model

cd unicorn_model
./setup.sh

source ~/ws_gazebo_roboracer/devel/setup.bash
```

## Quick Start

```bash
# Single vehicle with keyboard teleop on ICRA racetrack
roslaunch unicorn_model icra_racetrack.launch

# Single vehicle (default world)
roslaunch unicorn_model unicorn.launch

# Without teleop
roslaunch unicorn_model unicorn.launch teleop:=false

# Two vehicles
roslaunch unicorn_model dual_unicorn.launch

# Headless (no GUI)
roslaunch unicorn_model unicorn.launch gui:=false
```

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/livox/lidar` | `sensor_msgs/PointCloud2` | LiDAR point cloud (10Hz) |
| `/livox/imu` | `sensor_msgs/Imu` | IMU data (200Hz) |
| `/glim_ros/base_pose` | `geometry_msgs/PoseStamped` | Ground truth pose (rear axle) |
| `/glim_ros/base_odom` | `nav_msgs/Odometry` | Ground truth odometry |
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity command input |

## TF Tree

```
map -> base_link -> chassis
                 -> livox_mid360 -> imu
                                 -> lidar
```

## Project Structure

```
unicorn_model/
├── config/          # Controller PID parameters
├── docs/            # Detailed documentation
├── launch/          # Launch files
├── meshes/          # 3D models (STL, DAE)
├── scripts/         # Teleop, twist-to-joint bridge, GT publisher, visualization
├── thirdparty/      # Livox-SDK, LiDAR simulation plugin, ROS driver
├── urdf/            # Vehicle URDF/Xacro model
└── world/           # Gazebo world files
```

## Documentation

- [ROS Topics & TF Tree](docs/topics.md)
- [Vehicle Parameters](docs/vehicle-parameters.md)

## Dependencies

`setup.sh` installs all dependencies automatically.

| Package | Source | Description |
|---------|--------|-------------|
| [Livox-SDK](https://github.com/Livox-SDK/Livox-SDK) | thirdparty | C++ SDK (built from source) |
| [livox_laser_simulation](https://github.com/lzhhh6161/livox_laser_simulation) | thirdparty | Gazebo LiDAR plugin |
| [livox_ros_driver](https://github.com/Livox-SDK/livox_ros_driver) | thirdparty | ROS driver |
| libapr1-dev, libpcl-dev, ros-noetic-pcl-ros | apt | System libraries |

## License

MIT
