# TODO

## Vehicle Model
- [ ] Visual mesh가 collision cylinder와 안 맞음 — STL mesh를 실차 치수에 맞게 교체하거나 cylinder visual로 통일
- [ ] Steering hinge `rpy="0 1.5708 0"` hack — 트랙폭 넓히면 바퀴 캠버 발생. 근본적으로 joint 구조 재설계 필요
- [ ] Suspension 미구현 — 실차는 oil-filled shock (0.45 kgf/cm spring) 있음
- [ ] CoG 위치 미검증 — 현재 0.1477m (원래 모델 그대로), 실차 측정 필요

## Sensor
- [ ] LiDAR per-point timestamp 없음 — PointCloud2(XYZ)에 time 필드가 없어서 motion compensation 불가
- [ ] LiDAR mount 위치 실차 측정 후 반영 필요
- [ ] IMU extrinsic (T_lidar_imu) 실차 측정값으로 업데이트

## Autonomous Racing
- [ ] ICRA 트랙 global_waypoints.json 생성 — 레이스라인 없음
- [ ] `/car_state/odom`, `/car_state/pose` 토픽 브릿지 — gt_pose_publisher → car_state 변환
- [ ] Frenet conversion server 연동
- [ ] State machine + planner + controller 풀스택 테스트

## Simulation Quality
- [ ] Gazebo 물리 진동 — 정지 상태 IMU acc ±1.7 m/s² 진동 (물리엔진 한계)
- [ ] Contact parameter 튜닝 — kp/kd/mu 최적값 찾기
- [ ] Steering PID 튜닝 — 고속 주행 시 조향 응답성
