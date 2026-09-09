# 인지·판단·제어 알고리즘

이 문서는 최종 코드와 `scripts/start_integrated_drive_container.sh`에 기록된 대회 파라미터를 기준으로 작성했습니다.

## 인지

### 카메라 보정과 차선 기하

Xycar의 어안 카메라는 OpenCV fisheye 모델로 왜곡을 보정합니다. `Camera_Publisher.py`와 `topic_stamper.py`가 calibration matrix와 distortion coefficients로 rectification map을 미리 만들고 각 프레임에 적용합니다.

차선 파이프라인은 다음 순서로 기하 정보를 계산합니다.

1. YOLOv10n `center_line` 모델로 중심선 후보를 검출합니다.
2. `cv2.getPerspectiveTransform`과 `warpPerspective`로 도로 영역을 bird's-eye view로 변환합니다.
3. Canny edge와 `HoughLinesP`로 차선 선분을 찾습니다.
4. 검출점을 2차 또는 3차 다항식으로 근사해 경로 중심, heading과 curvature를 구합니다.
5. 양쪽 외곽선의 관계와 이전 프레임을 함께 사용해 한 프레임의 잘못된 차선 역할 전환을 억제합니다.

차선 YOLO 입력은 320 px, scene YOLO 입력은 640 px입니다. 최종 대회 launcher의 상한은 각각 15 Hz와 10 Hz로, 조향에 필요한 차선 지연과 여러 class를 구분해야 하는 장면 인지 해상도 사이의 균형을 잡았습니다. 재현용 node default는 lane 12 Hz, scene 4 Hz이며 [architecture 문서](ARCHITECTURE.md#perception-rates-default와-최종-운용값)를 함께 봐야 합니다.

### 신호·콘·장애물 인지

scene 모델은 `cone`, `dynamic`, `green`, `left`, `red`, `static` 여섯 class를 사용합니다. 실행 파라미터의 직접 판정 confidence는 red 0.30, left 0.60, green 0.50이며, left 신호는 진행 허가 신호로 처리합니다.

LiDAR 콘 인지는 다음 조건을 사용합니다.

- 유효 거리: 0.18–1.50 m
- DBSCAN: `eps=0.04 m`, `min_samples=3`
- 콘 후보 최대 지름: 0.30 m
- angular sector에서 첫 좌·우 콘을 찾고 가까운 후보를 따라 각각의 열을 성장
- 한쪽 열이 일시적으로 비면 virtual cone을 두어 중앙 경로의 불연속 완화
- 좌우 콘 쌍의 중점을 `CubicSpline`으로 보간하고 100개 경로점 생성

콘 진입은 LiDAR만 보고 즉시 전환하지 않습니다. `cone_entry_fusion_node`가 외부 보정값으로 LiDAR cluster를 어안 영상에 투영하고 YOLO cone box와 결합합니다. 기본값은 양쪽 콘과 최소 2개의 fused cluster를 요구하며, 최대 결합 지연은 0.30 s입니다. 사전 감지는 최근 3회 중 2회가 맞는지 확인해 속도를 제한합니다.

일반 장애물은 연속한 LaserScan 점들을 O(N)으로 묶고 cluster 거리의 50 percentile, 즉 중앙값을 대표 거리로 사용합니다. 영상 box와 LiDAR 방향을 연결한 뒤 다음 조건으로 시간상 같은 대상을 유지합니다.

- 카메라와 LiDAR timestamp 차이 최대 0.15 s
- 거리 jump 및 range-rate gate
- image offset jump gate
- bounding-box IoU 또는 중심 거리 gate
- 너무 가까운 cluster는 영상 box의 크기와 하단 위치를 추가 확인

### 최신성 관리

각 파이프라인은 원본 timestamp를 유지합니다. `frame_router`는 큐에 오래된 프레임을 쌓지 않고 최신 프레임 중심으로 전달하며, mission/controller는 정해진 timeout을 넘은 상태나 명령을 사용하지 않습니다. `tools/freshness_analysis`와 `tools/yolo_optimization`은 sensor-to-command 지연, 처리율, CPU/GPU 자원과 frame drop을 비교하는 데 사용했습니다.

## 판단

### 미션 상태기계

최상위 상태는 `LANE`와 `CONE`입니다. 시작 시 `LANE`이고, 콘의 사전 감지·융합 확인·연속 확인 조건이 맞으면 `CONE`으로 전환합니다. 최소 체류 시간과 exit hold를 둬 경계에서 모드가 반복 전환되는 현상을 막습니다.

안전 우선순위는 `PAUSED > RED/STOP > CONE > LANE/OVERTAKE`입니다. 적색은 2프레임 확인 후 래치되며 녹색 또는 좌회전 신호가 확인될 때까지 해제하지 않습니다. 센서가 오래됐거나 VESC telemetry 준비 조건을 만족하지 못하면 최종 출력은 0입니다.

### 정적·동적 장애물

장애물 회피는 class, 상대 거리, 도로 상태와 최근 프레임의 일치 여부를 함께 확인합니다. 같은 box는 IoU로 연결하고, 현재 장애물의 반대편 차선을 목표로 설정합니다. 최종 실행 파라미터는 다음과 같습니다.

| 대상 | 속도 명령 | 감속/판정 거리 | 차선 전환 거리 | 이벤트 유지 |
|---|---:|---:|---:|---:|
| 동적 장애물 | 26 | 4.0 m | 4.0 m | 2.0 s |
| 정적 장애물 | 18 | 3.0 m | 1.5 m | 1.2 s |

검출이 사라진 뒤에도 class별 clear frame 수를 만족해야 다음 이벤트를 받을 수 있어 동일 장애물의 중복 트리거를 줄입니다. 지름길 제어가 진행 중일 때는 장애물 전이가 지름길 상태를 덮어쓰지 않도록 차단합니다.

### 좌회전 지름길

`ShortcutTurnState` 기반 상태기계가 좌회전 신호, 진입 도로 형상, 횡단선, 회전 시간, 출구 정렬을 단계별로 확인합니다. 좌회전 신호는 최근 5프레임 중 3회, 횡단선은 최근 3프레임 중 2회, 출구 정렬은 최근 5프레임 중 2회를 요구합니다. 선분 계열과 중심 경로를 분석해 단일 차선의 원근 간격을 실제 좌측 분기로 오인하는 경우를 줄였습니다.

## 제어

### 차선 Stanley 제어

차선 모드는 heading error와 cross-track error(CTE)를 결합합니다.

```text
steering = heading_weight × heading_error + atan2(k × CTE, 3.0)
```

`k`와 heading weight는 속도와 도로 상태에 따라 바뀝니다. 속도 명령 4에서 12 사이를 선형 보간하며, 직선의 `k`는 1.00→0.65, 곡선은 1.20→0.90, 장애물 회피는 1.80을 사용합니다. 고속 직선에서는 heading 반응을 낮추고 곡선에서는 다시 높여 노이즈와 회전 지연을 함께 관리합니다.

현재 곡률과 preview 곡률을 median-filtered band로 분류해 기본 속도를 정합니다.

| 도로 상태 | 속도 명령 |
|---|---:|
| 직선 | 30 |
| 강·중·약 곡선 | 13 |
| 직선 복귀 | 20 |

조향 변화율은 직선 200, 곡선 300, 방향 반전 360 command-unit/s로 제한할 수 있으며 속도도 slew-rate를 적용합니다. CTE·heading innovation이 비정상적으로 크거나 Hough와 중심 곡선이 충돌하면 이전의 안정된 상태와 recovery 조건을 사용합니다.

### 콘 Pure Pursuit

최종 설정은 콘 경로에 Pure Pursuit를 사용합니다. 차량 좌표계의 spline 경로에서 lookahead 거리 앞의 목표점을 고르고, 그 점까지의 기하로 조향각을 계산합니다. 콘 구간 속도는 명령 7로 고정해 인지 프레임 사이의 경로 변화에 대응할 시간을 확보합니다.

코드에는 대안으로 cone Stanley와 preview control도 포함되어 있습니다. preview control은 경로의 세 점으로 곡률을 계산하고 다음 항을 조합합니다.

```text
steering = atan(wheelbase × preview_curvature) + heading feedback + CTE feedback
speed ∝ sqrt(lateral_acceleration_limit / |curvature|)
```

대안 구현은 실험과 회귀 테스트에 남겨 두었으며 최종 실행 스크립트의 `cone_controller`는 `pure_pursuit`입니다.

### VESC 출력

`xycar_motor_native`는 선택된 `[angle, speed]`를 VESC duty/speed 명령으로 변환합니다. startup handshake, reconnect 주기, telemetry timeout을 확인한 뒤 출력하며 준비되지 않았거나 command freshness를 잃으면 0 명령을 보냅니다.

