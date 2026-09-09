# xycar_gz_sim

ROS 2 Humble + Gazebo Sim Harmonic용 Xycar 통합 패키지이다. Gazebo Classic 플러그인은 사용하지 않으며 기존 트랙 월드와 `planningcode`는 수정하지 않는다.

## 분석 결과

차량 모델은 팀의 실차 camera/LiDAR/motor rosbag, 주행 코드와 `worlds/kookmin_track_from_dxf.world.sdf`를 분석해 구성했다. 원본 bag은 저장소에 포함하지 않는다.

### 팀 명령 인터페이스

- `planning_command_gate` 출력: `/cmd/speed`, `/cmd/steer`, 타입 `std_msgs/msg/Float32`, reliable/volatile depth 10
- `planning_control`의 `/cmd/steer`: ROS 좌표계 논리 조향각 [rad], 기본 제한 ±0.52 rad, 30 Hz
- `planning_speed`의 속도 출력 주기: 20 Hz
- 어댑터 출력: `/xycar_motor`, `std_msgs/msg/Float32MultiArray`, 배열 `[steer, speed]`, layout empty

세 bag 모두 직진에서 `/xycar_motor`의 첫 원소가 `-7.0`이었다. 사용자 실측 정의 `physical raw +7 = 직진`과 함께 만족시키기 위해 물리 raw와 실차 토픽 경계 부호를 분리했다. logical steer 0은 `/xycar/physical_raw_steer=+7`이고, 기본 `motor_topic_steer_sign=-1`을 거쳐 `/xycar_motor=[-7, speed]`가 된다. 하드웨어 드라이버 소스가 제공되면 이 경계 부호를 다시 확인해야 한다.

### rosbag 요약

| bag | 실제 motor 값 | motor 유지 구간 | 전체 bag |
|---|---|---:|---:|
| 01, speed 5 | `[-7, 5]` 104회 | 10.272 s | 18.178 s |
| 02, speed 10 | `[-7, 10]` 54회 | 5.282 s | 12.969 s |
| 03, speed 15 | `[-7, 15]` 35회 | 3.389 s | 9.481 s |

`/xycar_motor`는 약 10 Hz, reliable, volatile이다. 명령 유지시간은 실측 5 m 시간 9.84/4.90/3.31 s와 각각 +0.432/+0.382/+0.079 s 차이이다.

### 센서 인터페이스

- `/image_raw`: `sensor_msgs/msg/Image`, 640×480, `rgb8`, step 1920, frame `usb_cam`, 약 30 Hz, reliable/volatile
- `/camera_info`: `sensor_msgs/msg/CameraInfo`, frame `usb_cam`, 약 30 Hz, reliable/volatile. D/K/R/P는 `config/xycar.yaml`에 bag 값 그대로 저장
- `/scan`: `sensor_msgs/msg/LaserScan`, frame `laser_frame`, 약 9.66 Hz, best_effort/volatile, 500 samples, -π~+π, 0.1~16.0 m
- IMU는 모델과 bridge에 포함하지 않음

카메라와 LiDAR mount pose는 자료에 없으므로 `추정값 — 실측 필요`로 표시했다. 센서 bag을 차량 마찰, slip 또는 가감속 추정에 사용하지 않았다.

차량 형상과 미측정 동역학 초기값은 `config/model.yaml`, ROS 토픽과 보정 LUT는 `config/xycar.yaml`에 분리되어 있다.

## 차량과 조향

- rear axle 중심을 `base_link` 원점으로 사용
- 앞바퀴는 독립 steering joint, 뒷바퀴만 구동 joint로 등록
- Gazebo Harmonic 기본 `gz::sim::systems::AckermannSteering` 사용
- raw↔곡률은 좌우 비대칭 LUT와 선형보간 사용
- 총 질량 4.1 kg: body 3.50 + wheels 0.56 + knuckle/sensor links 0.04 kg
- 관성은 body를 직육면체, wheel을 solid cylinder로 보고 xacro 주석의 표준 식으로 계산

raw +40, 반경 0.5525 m와 wheelbase/track을 순수 Ackermann으로 동시에 적용하면 안쪽 바퀴가 약 39.7°여야 한다. 이는 “최대 앞바퀴각 약 30°”와 동시에 성립하지 않는다. 현재 모델은 반경 재현 검증을 위해 joint limit 0.72 rad를 사용하고 명목 실측 최대각 0.523599 rad를 별도로 기록한다.

## 빌드

저장소 루트에서 빌드한다.

```bash
source /opt/ros/humble/setup.bash
colcon --log-base simulation/log build --symlink-install \
  --base-paths simulation/xycar_gz_sim simulation/team_code/track_drive \
  --build-base simulation/build \
  --install-base simulation/install
source simulation/install/setup.bash
```

## 기존 트랙에서 실행

```bash
ros2 launch xycar_gz_sim xycar.launch.py use_sim:=true
```

실행기는 같은 world를 사용하는 Gazebo 서버가 이미 있으면 두 번째 실행을 차단한다.
launch를 끝낼 때는 실행한 터미널에서 `Ctrl-C`를 한 번 누른다. 감시 프로세스가 Gazebo
서버와 GUI의 전체 프로세스 그룹을 함께 종료하므로 서버가 백그라운드에 남지 않는다.

GUI 시작 카메라는 기본 spawn 위치 `(0, -3.5)`의 0.62 m 차량이 바로 보이도록 가까이
배치했다. 파란 차체·은색 후드·캐빈·roll cage가 보이면 최신 Y2 외형이 적용된 것이다.
주행 중 차량이 화면 밖으로 나가면 Entity Tree의 `xycar`를 우클릭하고 `Follow`를 선택한다.

중복 서버 오류가 표시되면 실행 중인 Gazebo를 먼저 확인하고 정상 종료한다.

```bash
pgrep -af 'gz sim'
```

서로 다른 터미널에서 시뮬레이션 launch를 두 번 실행하지 않는다. 같은 world의 여러
서버는 동일한 Gazebo Transport 토픽에 같은 이름의 `xycar` 상태를 발행해 차량이
깜빡이거나 순간이동하는 것처럼 보이게 한다.

GUI 없이 센서까지 실행:

```bash
ros2 launch xycar_gz_sim xycar.launch.py use_sim:=true gui:=false
```

실차 어댑터 모드에서는 Gazebo, bridge, sim 센서를 실행하지 않는다.

```bash
ros2 launch xycar_gz_sim xycar.launch.py use_sim:=false
```

기존 알고리즘은 계속 `/cmd/speed`, `/cmd/steer`를 발행하고 `/image_raw`, `/camera_info`, `/scan`을 구독하면 된다. remap이나 알고리즘 코드 수정은 필요하지 않다.

## 통합 주행 코드 연결

Gazebo와 `ros2_ws`의 통합 주행 스택은 `/image_raw`, `/camera_info`, `/scan`,
`/cmd/speed`, `/cmd/steer`를 공통으로 사용한다. 전체 인지·미션 코드를 함께 실행할
때는 저장소 루트의 `docs/SIM_TO_REAL.md`와 `docs/OPERATIONS.md`를 따른다.

기본 시작점에서 64초 이상 연속 주행과 우회전 곡선 통과를 확인했다. 오른쪽 S자
후반의 급한 좌회전에서는 차선을 잃고 안전 정지하며, 이 제한과 필요한 실측 보정은
`docs/SIM_TO_REAL.md`에 기록했다.

초기 `track_drive` 코드를 Gazebo 차량과 연결해 비교할 때는 다음 launch를 사용한다.

```bash
ros2 launch xycar_gz_sim team_lane_sim.launch.py use_sim_time:=true gui:=true
```

## 키보드로 직접 주행

키보드 운전은 launch 프로세스와 터미널 입력이 섞이지 않도록 터미널 두 개를 사용한다.

터미널 1 — 기존 트랙과 차량 실행:

```bash
source /opt/ros/humble/setup.bash
source simulation/install/setup.bash
ros2 launch xycar_gz_sim xycar.launch.py use_sim:=true gui:=true
```

터미널 2 — 키보드 노드 실행:

```bash
source /opt/ros/humble/setup.bash
source simulation/install/setup.bash
ros2 run xycar_gz_sim keyboard_teleop
```

키 조작:

| 키 | 동작 |
|---|---|
| `W` / `↑` | speed command 1 증가 |
| `S` / `↓` | speed command 1 감소 |
| `A` / `←` | 왼쪽으로 logical steer 2° 증가 |
| `D` / `→` | 오른쪽으로 logical steer 2° 증가 |
| `1`, `2`, `3` | speed command 5, 10, 15 즉시 선택 |
| `Space` | 속도만 0, 현재 조향 유지 |
| `C` | 조향 중앙. logical 0 → physical raw +7 |
| `X` | 속도 0, 조향 중앙 |
| `H` | 도움말 다시 표시 |
| `Q` / `Ctrl-C` | 정지·중앙 복귀 후 종료 |

터미널 상태 줄에는 다음 값이 실시간 표시된다.

```text
speed cmd=+5.0 | logical steer=+4.0 deg | speed=+0.508 m/s |
physical raw=-5.92 | wheel L/R=+4.1/+3.9 deg
```

속도와 조향 증분은 실행할 때 변경할 수 있다.

```bash
ros2 run xycar_gz_sim keyboard_teleop --ros-args \
  -p speed_step:=2.0 \
  -p steer_step_deg:=1.0
```

음수 속도는 후진할 수 있도록 열어 두었지만 실차 후진 매핑은 **미측정값**이다.
키보드 운전 중에는 팀 알고리즘의 command gate를 동시에 실행하지 않는다. 두 노드가
동시에 `/cmd/speed`, `/cmd/steer`를 발행하면 명령이 번갈아 적용될 수 있다.

## Y2 스타일 외형

첨부된 Y2 모델 사진을 참고해 기존 단일 직육면체 외형을 다음 visual 요소로 교체했다.

- 검은 하부 섀시와 전·후 범퍼
- 파란 좌우 패널과 은색 스트라이프
- 경사진 은색 후드와 파란 hood insert
- 캐빈, 반투명 windshield, roof와 roll cage
- 전조등·후미등과 붉은 suspension spring
- 타이어의 은색 hub 및 파란 hub cap
- 카메라 body/lens와 LiDAR base/head

외형은 CAD mesh가 아닌 Xacro primitive 조합이다. 실측 wheelbase, track, wheel 크기,
총 질량과 관성은 바꾸지 않았고 차량 전체 폭 0.320 m는 계속 **추정값 — 실측 필요**다.

## 검증 launch

각 시험은 독립적으로 실행하며 결과를 출력한 뒤 자동 종료한다. 5 m 시간은 PC의
실시간 배율과 분리하기 위해 Gazebo `/clock` 기준으로 측정한다.

```bash
ros2 launch xycar_gz_sim xycar_test.launch.py test_case:=straight_zero
ros2 launch xycar_gz_sim xycar_test.launch.py test_case:=sensor_interface
ros2 launch xycar_gz_sim xycar_test.launch.py test_case:=speed_5
ros2 launch xycar_gz_sim xycar_test.launch.py test_case:=speed_10
ros2 launch xycar_gz_sim xycar_test.launch.py test_case:=speed_15
ros2 launch xycar_gz_sim xycar_test.launch.py test_case:=turn_p20
ros2 launch xycar_gz_sim xycar_test.launch.py test_case:=turn_p30
ros2 launch xycar_gz_sim xycar_test.launch.py test_case:=turn_p40
ros2 launch xycar_gz_sim xycar_test.launch.py test_case:=turn_m20
ros2 launch xycar_gz_sim xycar_test.launch.py test_case:=turn_m30
ros2 launch xycar_gz_sim xycar_test.launch.py test_case:=turn_m40
```

센서 QoS 확인:

```bash
ros2 topic info -v /image_raw
ros2 topic info -v /camera_info
ros2 topic info -v /scan
ros2 topic hz /image_raw
ros2 topic hz /scan
```

이 작업 환경에서 확인한 결과:

- Python 보정 수학 단위시험: 14/14 PASS
- `straight_zero`: `logical 0 -> physical raw +7.000`, bag 호환 motor raw `-7.000`, PASS
- `sensor_interface`: Image/CameraInfo/LaserScan 타입·크기·frame 일치, PASS
- `speed_15`: Gazebo 5 m 3.999 s, 실차 3.310 s, +0.689 s. 미측정 가속도 파라미터 튜닝 필요
- `turn_p40`: Gazebo 0.5474 m, 실차 0.5525 m, -0.0051 m
- `turn_m40`: Gazebo 0.8042 m, 실차 0.8200 m, -0.0158 m

마지막 두 결과가 동일한 절댓값 raw 40에서도 좌·우 비대칭 반경을 재현함을 확인한다.

## 추가 실측이 필요한 값

- 차량 전체 폭과 정확한 body 형상/무게중심
- 카메라와 LiDAR의 base_link 기준 6-DoF pose
- raw +40에서 좌우 개별 실제 앞바퀴각 및 회전반경 재측정
- 가속도, 감속도, 정지거리, jerk
- 조향 속도, 응답 지연, 모터 지연
- 종·횡 타이어 마찰계수와 slip
- command 1~3 및 후진 속도 매핑
- `/xycar_motor` 직전 하드웨어 드라이버의 조향 부호 변환
