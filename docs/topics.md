# ROS Topics

| Topic | Type | 설명 |
|-------|------|------|
| `/livox/lidar` | `sensor_msgs/PointCloud2` | Mid-360 포인트클라우드 (10Hz, 20000pts/frame) |
| `/livox/imu` | `sensor_msgs/Imu` | 내장 IMU (200Hz) |
| `/odom` | `nav_msgs/Odometry` | Ground truth odometry |
| `/cmd_vel` | `geometry_msgs/Twist` | 속도 명령 |
| `/unicorn/joint_states` | `sensor_msgs/JointState` | 관절 상태 |
| `/unicorn/*/command` | `std_msgs/Float64` | 개별 관절 제어 |

## TF Tree

```
base_link
├── chassis
│   ├── left_rear_wheel
│   ├── right_rear_wheel
│   ├── left_steering_hinge → left_front_wheel
│   └── right_steering_hinge → right_front_wheel
├── base_footprint
└── livox_mid360
```
