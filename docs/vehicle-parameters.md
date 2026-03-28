# Vehicle Parameters

## Chassis

**File**: `urdf/macros.xacro`

| Parameter | Location | Default | Description |
|-----------|----------|---------|-------------|
| Mass | `chassis_inertial_params` -> `mass` | 4.0 kg | Chassis weight |
| Center of gravity | `chassis_inertial_params` -> `origin xyz` | 0.1477 0 0 | CoG position (m) |

## Wheelbase / Track

**File**: `urdf/unicorn.urdf`

| Parameter | Default | Description |
|-----------|---------|-------------|
| Wheelbase | 0.325 m | `*_steering_hinge_joint` origin x |
| Track width | 0.2 m | `*_wheel_joint` / `*_steering_hinge_joint` origin y (±0.1) |
| Max steering angle | ±1.0 rad | `*_steering_hinge_joint` limit |

## Wheels

**File**: `urdf/macros.xacro`

| Parameter | Default | Description |
|-----------|---------|-------------|
| Mass | 0.34 kg | `*_wheels_inertial_params` -> mass |
| Radius | 0.05 m | `*_wheels_collision_geometry` -> cylinder radius |
| Width | 0.045 m | `*_wheels_collision_geometry` -> cylinder length |

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
