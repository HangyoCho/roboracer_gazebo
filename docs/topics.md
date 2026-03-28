# ROS Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/livox/lidar` | `sensor_msgs/PointCloud2` | Mid-360 point cloud (10Hz, 20000 pts/frame) |
| `/livox/imu` | `sensor_msgs/Imu` | Built-in IMU (200Hz) |
| `/glim_ros/base_pose` | `geometry_msgs/PoseStamped` | Ground truth pose at rear axle |
| `/glim_ros/base_odom` | `nav_msgs/Odometry` | Ground truth odometry at rear axle |
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity command |
| `/vesc/low_level/ackermann_cmd_mux/output` | `ackermann_msgs/AckermannDriveStamped` | Ackermann drive command |
| `/joy` | `sensor_msgs/Joy` | Joystick input |
| `/unicorn/joint_states` | `sensor_msgs/JointState` | Joint states |
| `/unicorn/*/command` | `std_msgs/Float64` | Individual joint commands |

## TF Tree

```
map
└── base_link (GT from gt_pose_publisher)
    ├── chassis
    │   ├── left_rear_wheel
    │   ├── right_rear_wheel
    │   ├── left_steering_hinge -> left_front_wheel
    │   └── right_steering_hinge -> right_front_wheel
    ├── base_footprint
    └── livox_mid360
        ├── imu
        └── lidar
```

## Frame Conventions

- **base_link**: Located at the rear axle center (= chassis origin)
- **livox_mid360**: LiDAR frame, offset (0.26, 0, 0.08) from base_link
- **imu / lidar**: Alias frames attached to livox_mid360 with identity transform
- **map**: World-fixed frame, published by gt_pose_publisher
