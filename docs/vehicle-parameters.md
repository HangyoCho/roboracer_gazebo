# 차량 파라미터 변경

## 차체

**파일**: `urdf/macros.xacro`

| 파라미터 | 위치 | 기본값 | 설명 |
|----------|------|--------|------|
| 질량 | `chassis_inertial_params` → `mass` | 4.0 kg | 차체 무게 |
| 무게중심 | `chassis_inertial_params` → `origin xyz` | 0.1477 0 0 | CoG (m) |

## 휠베이스 / 트랙

**파일**: `urdf/unicorn.urdf`

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| 휠베이스 | 0.325 m | `*_steering_hinge_joint` origin x |
| 트랙 폭 | 0.2 m | `*_wheel_joint` / `*_steering_hinge_joint` origin y (±0.1) |
| 최대 조향각 | ±1.0 rad | `*_steering_hinge_joint` limit |

## 바퀴

**파일**: `urdf/macros.xacro`

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| 질량 | 0.34 kg | `*_wheels_inertial_params` → mass |
| 반지름 | 0.05 m | `*_wheels_collision_geometry` → cylinder radius |
| 폭 | 0.045 m | `*_wheels_collision_geometry` → cylinder length |

## 마찰 / 접촉

**파일**: `urdf/unicorn.gazebo`

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| mu1, mu2 | 1.0 | Coulomb 마찰 계수 |
| kp | 10000000 | 접촉 강성 (N/m) |
| kd | 1.0 | 접촉 감쇠 |

## 컨트롤러 PID

**파일**: `config/unicorn_control.yaml`

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| 스티어링 PID | p=10 i=0 d=0 | 조향 위치 제어 |
| 휠 PID | p=10 i=0 d=0 | 구동 속도 제어 |

## Livox Mid-360

**파일**: `urdf/unicorn.urdf` (인라인 설정)

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `laser_max_range` | 40.0 m | 최대 감지 거리 |
| `samples` | 20000 | 프레임당 포인트 수 |
| `update_rate` | 10 Hz | 스캔 주기 |
| `csv_file_name` | mid360-real-centr.csv | 스캔 패턴 |
| `publish_pointcloud_type` | 1 | 0=PointCloud, 1=PointCloud2(XYZ), 2=PointCloud2(XYZRTL), 3=CustomMsg |
| 마운트 위치 | x=0.1 y=0 z=0.08 | base_link 기준 |
