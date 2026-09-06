# 시스템 아키텍처

## 설계 기준

통합 주행 스택은 센서 입력, 인지, 미션 판단, 경로 추종, 구동기 출력을 분리합니다. 실차와 시뮬레이션은 동일한 토픽 계약을 사용하고, `mission_manager_node`가 각 제어기의 출력을 현재 미션과 안전 상태에 따라 선택합니다.

## 데이터 흐름

### 센서와 프레임 분배

1. `usb_cam` 또는 Gazebo 카메라가 `/image_raw`와 `/camera_info`를 발행합니다.
2. `frame_router`는 가장 최근 프레임을 차선용·장면용 입력으로 나눕니다. 기본 구성은 새 프레임 도착에 반응하는 event-driven 방식이며 차선 입력은 최대 15 Hz, 장면 입력은 최대 10 Hz입니다.
3. Xycar LiDAR 또는 Gazebo bridge가 `/scan`을 발행하고 `scan_rotator`가 장착 방향을 보정합니다.

### 영상 인지

- lane pipeline: 320 px YOLO 입력에서 `center_line` 후보를 검출하고 `centerline_tracer`와 `Lane_Detector`가 BEV 좌표의 중심선·외곽선을 추정합니다.
- scene pipeline: 640 px YOLO 입력에서 `cone`, `dynamic`, `green`, `left`, `red`, `static`을 검출합니다. 코드 내부에서는 `dynamic`을 `obstacle_vehicle`로 정규화할 수 있습니다.
- 신호, 장애물, 지름길 후보는 source timestamp와 함께 전달되어 오래된 검출을 판단에서 제외합니다.

### LiDAR와 센서 융합

- 콘 구간: 전방 점을 DBSCAN으로 묶은 뒤 지름과 각도 조건으로 콘 후보를 남깁니다. 좌우 열을 성장시키고 각 쌍의 중점을 연결해 주행 경로를 만듭니다.
- 콘 진입: LiDAR cluster를 어안 카메라 모델로 영상에 투영하고 YOLO cone box와 겹치는지 확인합니다.
- 일반 장애물: 연속 scan cluster의 중앙값 거리를 사용합니다. 영상 box 방향과 LiDAR 각도를 연결한 뒤 거리·화면 위치·IoU의 시간 연속성 조건으로 같은 대상을 추적합니다.

### 판단과 제어

`mission_manager_node`의 기본 우선순위는 다음과 같습니다.

1. pause 또는 명시적 stop
2. 적색 신호와 안전 정지 조건
3. `CONE` 모드 제어 명령
4. `LANE` 모드의 차선 추종과 장애물 회피

`LANE`에서는 `Integrated_Stanley_Controller`, `CONE`에서는 기본적으로 `pure_pursuit_node`의 출력을 선택합니다. 모든 명령은 freshness와 구동기 준비 상태를 확인한 뒤 VESC 어댑터로 전달됩니다.

## 주요 노드

| 패키지 | 노드/모듈 | 책임 |
|---|---|---|
| `cam` | `frame_router` | 최신 카메라 프레임 분배와 처리율 제한 |
| `cam` | `yolo_node` | lane/scene YOLO 추론과 class별 confidence 적용 |
| `cam` | `Lane_Detector` | BEV, Hough, 차선 역할 추적, CTE·heading·curvature 산출 |
| `cam` | `Integrated_Stanley_Controller` | 차선 Stanley 조향과 곡률 기반 속도 계획 |
| `cam` | `shortcut_left_turn_node` | 좌회전 지름길의 단계별 경로 생성과 전이 |
| `mission_cone_drive` | `lidar_dbscan_preprocessing_node` | LiDAR 콘 clustering과 후보 필터링 |
| `mission_cone_drive` | `lidar_dbscan_path_planning_node` | 좌우 콘 열과 중앙 spline 경로 생성 |
| `mission_cone_drive` | `cone_entry_fusion_node` | YOLO cone box와 LiDAR cluster 결합 |
| `mission_cone_drive` | `mission_manager_node` | 모드 전환, 신호·장애물 정책, 명령 선택, fail-safe |
| `mission_cone_drive` | `pure_pursuit_node` | 콘 중앙 경로 추종 |
| `xycar_motor_native` | motor adapter | 논리 조향·속도 명령을 VESC 경계 형식으로 변환 |
| `xycar_gz_sim` | vehicle and adapters | Ackermann 차량, 센서, 실측 LUT 기반 인터페이스 제공 |

## 핵심 토픽 계약

| 토픽 | 타입/의미 | 생산자 → 소비자 |
|---|---|---|
| `/image_raw` | `sensor_msgs/Image` | camera/Gazebo → frame router |
| `/camera_info` | `sensor_msgs/CameraInfo` | camera/Gazebo → calibration consumers |
| `/scan` | `sensor_msgs/LaserScan` | LiDAR/Gazebo → scan rotation and clustering |
| `/cmd/speed` | `std_msgs/Float32` | planner → real/sim drive adapter |
| `/cmd/steer` | `std_msgs/Float32` | planner → real/sim steering adapter |
| `/xycar_motor` | `[steer, speed]` | adapter → vehicle driver |

## 실행 백엔드

`integrated_drive.launch.py`는 host와 container 실행 경로를 지원합니다. `scripts/start_integrated_drive_container.sh`는 `XYCAR_EXECUTION_BACKEND`로 이를 선택합니다. 공개 저장소에는 팀 컨테이너를 재구성하는 Dockerfile이 없으므로, 해당 이미지를 보유하지 않은 환경에서는 `host`를 사용합니다.

