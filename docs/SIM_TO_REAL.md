# Sim-to-Real 개발 기록

## 목표

Gazebo에서 검증한 알고리즘을 실차에 옮길 때 주행 노드의 토픽과 조향 기준이 바뀌지 않도록 구성했습니다. 시뮬레이터와 하드웨어 드라이버가 같은 입력·출력 계약을 제공하고, 차이는 adapter와 calibration 파일에서 처리합니다.

## 공통 인터페이스

| 기능 | 인터페이스 | 실차 | Gazebo |
|---|---|---|---|
| 카메라 | `/image_raw`, `/camera_info` | Xycar/USB camera | camera sensor bridge |
| LiDAR | `/scan` | YDLidar | GPU/2D lidar bridge |
| 속도 | `/cmd/speed` | VESC adapter | speed LUT adapter |
| 조향 | `/cmd/steer` | VESC/motor adapter | Ackermann adapter |
| 최종 명령 | `/xycar_motor` | physical driver | compatibility topic |

알고리즘 노드는 이 경계 안쪽에서 차량이 실제인지 simulated인지 알 필요가 없습니다.

## 차량 모델

- `base_link`: rear axle 중심
- steering: 앞바퀴 독립 joint와 Gazebo `AckermannSteering` system
- drive: 뒷바퀴 구동
- 질량: 4.1 kg(차체 3.50 kg, 바퀴 0.56 kg, knuckle/sensor link 0.04 kg)
- 관성: 직육면체 차체와 solid-cylinder wheel의 표준식으로 계산
- wheelbase/track/wheel size: `simulation/xycar_gz_sim/config/model.yaml`

차량 외형은 Xacro primitive로 구성했습니다. 외형은 센서 가시성과 차량 방향을 확인하기 위한 것으로 CAD 형상 정밀도를 뜻하지 않습니다.

## 실측 LUT

조향은 단일 비례식 대신 좌·우 비대칭 raw↔curvature LUT를 선형 보간합니다. 실측 자료에서 logical steer 0은 physical raw `+7`에 해당했고, 기존 `/xycar_motor` 기록에서는 토픽 경계 부호를 거쳐 `-7`이 직진으로 나타났습니다. 이 부호 분리는 `config/xycar.yaml`에 기록했습니다.

속도 명령과 차량 속도도 실측점을 보간합니다. 예를 들어 speed command 4는 약 0.399 m/s, 25는 약 2.222 m/s입니다. 측정 범위 밖의 값을 실제 값처럼 추정하지 않도록 문서와 teleop 기본값을 제한했습니다.

## 코스 생성

국민대 코스 도면의 닫힌 polyline 125개를 분석해 `simulation/worlds/kookmin_track_from_dxf.world.sdf`를 만들었습니다. 사용한 변환은 1 DXF unit = 1 mm = 0.001 m이며, 회색 도로 0.80 m와 양쪽 흰 경계 0.05 m를 합쳐 총 폭 0.90 m로 구성했습니다.

원본 DXF는 배포하지 않습니다. 생성된 world, mesh와 생성 스크립트만 남겨 결과를 검토할 수 있게 했습니다.

## 검증 결과와 남은 보정

Gazebo에서 기본 시작점 `(0, -3.5)` 부근에서 64초 이상 연속 주행했고 우회전 곡선을 통과했습니다. 오른쪽 S자 후반의 급한 좌회전에서는 차선을 잃고 안전 정지했습니다.

실측 보정상 최대 좌회전 반경은 0.820 m, 최대 우회전 반경은 0.5525 m로 비대칭입니다. 해당 S자 구간을 안정적으로 통과하려면 다음 항목이 필요합니다.

1. 카메라와 LiDAR mount pose 실측
2. S자 진입에서 바깥쪽 경로를 미리 선택하는 preview 정책
3. 좌회전 구간의 추가 steering LUT 측정
4. 동일 조명·속도 조건의 rosbag replay와 실차 A/B 비교

현재 구성은 차선을 잃었을 때 마지막 조향으로 계속 달리지 않고 속도 0을 출력합니다.

## 재현 절차

```bash
source /opt/ros/humble/setup.bash
colcon --log-base simulation/log build --symlink-install \
  --base-paths simulation/xycar_gz_sim simulation/team_code/track_drive \
  --build-base simulation/build \
  --install-base simulation/install
source simulation/install/setup.bash
ros2 launch xycar_gz_sim xycar.launch.py use_sim:=true gui:=true
```

초기 차선주행 코드와 adapter를 함께 비교하려면 다음 launch를 사용합니다.

```bash
ros2 launch xycar_gz_sim team_lane_sim.launch.py use_sim_time:=true gui:=true
```

카메라·LiDAR·차량만 확인하려면 `gui:=false`로 headless 실행할 수 있습니다. 키보드와 자율주행 노드는 둘 다 같은 명령 토픽을 쓰므로 동시에 실행하지 않습니다.
