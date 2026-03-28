# Vehicle Parameters

## Chassis

**File**: `urdf/macros.xacro`

| Parameter | Location | Default | Description |
|-----------|----------|---------|-------------|
| Mass | `chassis_inertial_params` -> `mass` | 1.70 kg | Chassis weight |
| Center of gravity | `chassis_inertial_params` -> `origin xyz` | 0.18 0 0 | CoG from rear axle (m) |

## Wheelbase / Track

**File**: `urdf/unicorn.urdf`

| Parameter | Default | Description |
|-----------|---------|-------------|
| Wheelbase | 360 mm | `*_steering_hinge_joint` origin x |
| Front track | 302 mm | `*_steering_hinge_joint` origin y (±0.151) |
| Rear track | 296 mm | `*_rear_wheel_joint` origin y (±0.148) |
| Max steering angle | ±0.52 rad (~30°) | `*_steering_hinge_joint` limit |

## Wheels

**File**: `urdf/macros.xacro`

| Parameter | Default | Description |
|-----------|---------|-------------|
| Mass | 0.12 kg | `*_wheels_inertial_params` -> mass |
| Radius | 55 mm | `*_wheels_collision_geometry` -> cylinder radius |
| Width | 26 mm | `*_wheels_collision_geometry` -> cylinder length |

## Friction / Contact

**File**: `urdf/unicorn.gazebo`

| Parameter | Default | Description |
|-----------|---------|-------------|
| mu1, mu2 | 1.5 | Coulomb friction coefficients |
| kp | 1,000,000 | Contact stiffness (N/m) |
| kd | 100.0 | Contact damping |
| minDepth | 0.001 | Minimum contact penetration depth |

## Joint Dynamics

**File**: `urdf/unicorn.urdf`

| Joint | Damping | Friction | Description |
|-------|---------|----------|-------------|
| Wheel joints (x4) | 0.02 | 0.0 | Rolling resistance |
| Steering joints (x2) | 0.02 | 0.0 | Steering damping |

## Controller PID

**File**: `config/unicorn_control.yaml`

| Controller | Type | PID | Description |
|------------|------|-----|-------------|
| Steering | JointPositionController | p=10 i=0 d=0.5 | Steering position control |
| Wheels | JointVelocityController | p=10 i=0 d=0 | Drive velocity control |

## Livox Mid-360

**File**: `urdf/unicorn.urdf` (inline config)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `laser_max_range` | 60.0 m | Maximum detection range |
| `samples` | 20000 | Points per frame |
| `update_rate` | 10 Hz | Scan rate |
| `csv_file_name` | mid360-real-centr.csv | Scan pattern file |
| `publish_pointcloud_type` | 1 | 0=PointCloud, 1=PointCloud2(XYZ), 2=PointCloud2(XYZRTL), 3=CustomMsg |
| Mount position | x=0.26 y=0 z=0.08 | Relative to base_link |
| IMU noise | 0.0 | Gaussian noise (disabled for sim) |
| LiDAR noise stddev | 0.001 | Range measurement noise |

## Overall Vehicle Specs

| Spec | Value |
|------|-------|
| Length | 560 mm |
| Width | 302 mm |
| Height | 160 mm |
| Wheelbase | 360 mm |
| Front Track | 302 mm |
| Rear Track | 296 mm |
| Total Weight | ~2.35 kg |
| Drive | 4WD |
| Shocks | Oil-filled 17mm, #600 silicone oil |
| Spring Rate | 0.45 kgf/cm |
