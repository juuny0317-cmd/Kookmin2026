# Sim-to-Real 개발 기록

## 목표와 범위

실차 한 대를 튜닝하는 동안 다른 팀원도 Gazebo에서 perception/control을 개발할 수 있도록 sensor·vehicle interface와 실측 calibration을 재현했다. 정확한 CAD나 steering mechanism 설계자료가 없는 차량이어서 geometry와 command response를 직접 측정했다.

이 결과는 완전한 digital twin이 아니다. **실측 기반 Sim-to-Real framework를 구축하고 vehicle interface와 geometry calibration까지 진행했으나, latency와 미측정 steering/sensor 특성 때문에 trajectory-level 일치에는 한계가 있다.**

## Interface boundary

| 기능 | Physical Xycar final stack | Gazebo verification |
|---|---|---|
| Camera | `/image_raw`, `/camera_info` | sensor adapter가 같은 토픽 제공 |
| LiDAR | `/scan` | sensor adapter가 같은 토픽 제공 |
| 팀 통합 명령 | Mission Manager → `/xycar_motor [steer, speed]` | `team_motor_adapter`가 `/team/xycar_motor_cmd`를 logical command로 변환 |
| Simulator logical command | 해당 없음 | `/cmd/speed`, `/cmd/steer` |
| Actuator | MotorCommandAdapter → VESC servo/duty | Ackermann + velocity command adapter |

`/cmd/speed`와 `/cmd/steer`는 Gazebo adapter의 논리 경계이며 실차 final VESC output이 아니다. 실차 end-to-end 경로는 `/xycar_motor → /commands/servo/position + /commands/motor/duty_cycle → VESC`다.

## Vehicle model과 calibration

- rear axle 중심 `base_link`
- front independent steering joints, rear-wheel drive
- wheelbase 0.355 m, front/rear track 0.250/0.266 m
- wheel radius 0.050 m, total mass 4.1 kg
- direction-specific raw steering↔curvature LUT
- measured speed command↔m/s LUT

전체 치수, 5 m 속도 표, 좌우 원주행 비대칭과 계산 근거는 [실차 calibration](REAL_VEHICLE_CALIBRATION.md)에 있다. vehicle width 0.320 m, camera/LiDAR mount pose, tire friction/slip, 축중, steering/motor delay는 미측정 또는 추정값으로 설정 파일에 표시했다.

## 검증 결과

| Check | Result | 해석 |
|---|---|---|
| Calibration math unit tests | 14/14 passed | LUT와 Ackermann 계산 regression |
| Straight zero | physical raw +7.000, bag-compatible topic raw -7.000 | sign boundary 확인 |
| Sensor interface | Image/CameraInfo/LaserScan type·size·frame passed | topic contract 확인 |
| raw +40 right radius | real 0.5525 m, Gazebo 0.5474 m, 0.9% abs error | 최대 right 반경 재현 |
| raw -40 left radius | real 0.8200 m, Gazebo 0.8042 m, 1.9% abs error | 최대 left 반경 재현 |
| command 15, 5 m | real 3.310 s, Gazebo 3.999 s, 20.8% time error | acceleration model 추가 tuning 필요 |
| Kookmin course | 64 s 이상 연속, right turn 통과 | 정성 주행 확인 |
| Sharp S-curve exit | late left에서 lane loss 후 speed 0 | 안전 정지 확인, 완주 실패 |

최대 조향 반경 오차가 작다는 사실은 LUT knot를 재현했다는 뜻이지 전체 궤적 정확도를 뜻하지 않는다. raw ±20/±30의 Gazebo circle result와 동일 조건 trajectory log가 없으므로 전체 sim-real error를 일반화하지 않는다.

## 남은 gap

- CAD와 정확한 steering linkage 정보 부재
- camera/LiDAR mount pose 오차
- 실차 steering asymmetry와 추가 knot 부족
- tire friction, slip, weight distribution uncertainty
- 실제 camera/inference/command latency 미반영
- steering/motor response delay 차이
- 조명과 영상 domain gap

## 재현

```bash
source /opt/ros/humble/setup.bash
colcon --log-base simulation/log build --symlink-install \
  --base-paths simulation/xycar_gz_sim simulation/team_code/track_drive \
  --build-base simulation/build \
  --install-base simulation/install
source simulation/install/setup.bash

# 차량·센서와 calibration test
ros2 launch xycar_gz_sim xycar.launch.py use_sim:=true gui:=true
ros2 launch xycar_gz_sim xycar_test.launch.py test_case:=turn_p40
ros2 launch xycar_gz_sim xycar_test.launch.py test_case:=turn_m40
ros2 launch xycar_gz_sim xycar_test.launch.py test_case:=speed_15

# 초기 팀 lane code 비교
ros2 launch xycar_gz_sim team_lane_sim.launch.py use_sim_time:=true gui:=true
```

## Simulation 미디어

[Kookmin course screenshot](../media/simulation/kookmin_gazebo_course.jpg)

위 screenshot과 표의 검증 수치는 서로 다른 증거이며, 이미지만으로 정량 성능을 주장하지 않는다.
