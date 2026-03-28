# Unicorn Racing Vehicle

<p align="center">
  <img src="meshes/chassis.dae" width="400" alt="Unicorn Vehicle"/>
</p>

Livox Mid-360 LiDAR를 장착한 1/10 스케일 자율주행 레이싱 차량의 Gazebo 시뮬레이션 패키지입니다.

## Features

- **Ackermann Steering** — 실차와 동일한 조향 기구학
- **Livox Mid-360** — 비정형 스캔 패턴의 PointCloud2 출력 (10Hz, 20k pts/frame)
- **IMU** — 200Hz 내장 IMU 센서
- **키보드 텔레옵** — 별도 터미널에서 즉시 조작 가능
- **멀티 에이전트** — 2대 동시 스폰 지원
- **커스텀 월드** — 레이싱 코스 맵 포함

## Prerequisites

- Ubuntu 20.04
- ROS Noetic
- Gazebo Classic 11

## Installation

```bash
# 워크스페이스에 클론
mkdir -p ~/ws_gazebo_roboracer/src && cd ~/ws_gazebo_roboracer/src
git clone <repo-url> unicorn_model

# 의존성 설치 + 빌드 (한 번에)
cd unicorn_model
./setup.sh

source ~/ws_gazebo_roboracer/devel/setup.bash
```

## Quick Start

```bash
# 단일 차량 (키보드 텔레옵 포함)
roslaunch unicorn_model unicorn.launch

# 텔레옵 없이
roslaunch unicorn_model unicorn.launch teleop:=false

# 2대 동시 스폰
roslaunch unicorn_model dual_unicorn.launch

# GUI 없이 (headless)
roslaunch unicorn_model unicorn.launch gui:=false
```

## Keyboard Control

launch 시 자동으로 텔레옵 터미널이 열립니다.

| 키 | 동작 |
|----|------|
| `w` | 전진 가속 |
| `s` | 후진 가속 |
| `a` | 좌회전 |
| `d` | 우회전 |
| `r` | 정지 |
| `q` | 종료 |

## Project Structure

```
unicorn_model/
├── config/          # 컨트롤러 PID 설정
├── docs/            # 상세 문서 (Topics, 파라미터 등)
├── launch/          # roslaunch 파일
├── meshes/          # 3D 모델 (STL, DAE)
├── scripts/         # 텔레옵, twist→joint 변환
├── thirdparty/      # 의존 패키지 (Livox-SDK, 플러그인, 드라이버)
├── urdf/            # 차량 URDF/Xacro 모델
└── world/           # Gazebo 월드 맵
```

## Documentation

- [ROS Topics & TF Tree](docs/topics.md)
- [차량 파라미터 변경](docs/vehicle-parameters.md)

## Dependencies

`setup.sh`가 자동으로 설치합니다.

| 패키지 | 구분 | 설명 |
|--------|------|------|
| [Livox-SDK](https://github.com/Livox-SDK/Livox-SDK) | thirdparty | C++ SDK (자동 빌드) |
| [livox_laser_simulation](https://github.com/lzhhh6161/livox_laser_simulation) | thirdparty | Gazebo LiDAR 플러그인 |
| [livox_ros_driver](https://github.com/Livox-SDK/livox_ros_driver) | thirdparty | ROS 드라이버 |
| libapr1-dev, libpcl-dev, ros-noetic-pcl-ros | apt | 시스템 라이브러리 |

## License

MIT
