# 빌드와 운용

## 환경

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10
- Gazebo Sim Harmonic(시뮬레이션)
- OpenCV, NumPy, SciPy, PyTorch/Ultralytics(인지 노드)

센서와 모터를 연결하기 전에 emergency stop이 가능한 상태에서 바퀴를 지면에서 띄워 방향과 정지 동작을 먼저 확인합니다.

## 통합 워크스페이스 빌드

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## 호스트 실행

저장소 루트에서 실행합니다.

```bash
XYCAR_EXECUTION_BACKEND=host ./scripts/start_integrated_drive_container.sh
```

스크립트는 기본적으로 자신의 `ros2_ws`를 찾습니다. 다른 workspace를 쓸 때만 환경 변수를 지정합니다.

```bash
XYCAR_WORKSPACE=/absolute/path/to/xycar_ws \
XYCAR_EXECUTION_BACKEND=host \
./scripts/start_integrated_drive_container.sh
```

## 컨테이너 실행

원래 대회 환경은 digest가 고정된 팀 image를 사용했습니다. 공개 저장소에는 이를 빌드하는 Dockerfile과 private base image가 없습니다. 이미지를 보유한 장비에서만 다음과 같이 지정합니다.

```bash
XYCAR_IMAGE='sha256:03acec7d3f671131801a951600d164566436e4a2b009fc109d755f2bd5a41627' \
./scripts/start_integrated_drive_container.sh
```

당시 절대 경로와 파라미터를 그대로 보존한 파일은 `scripts/reference/start_integrated_drive_container_original.sh`입니다. 일반 실행에는 portable script를 사용합니다.

## 시작과 중지

통합 launch는 `start_active:=false`로 기동합니다. 센서 토픽과 VESC telemetry가 정상인지 확인한 뒤 제공된 start/pause/resume/stop Trigger 서비스를 사용합니다. 실제 service name은 launch 출력과 다음 명령으로 확인합니다.

```bash
ros2 service list | grep integrated
```

종료는 launch를 실행한 터미널에서 `Ctrl-C`로 수행합니다. 재시작 전에 남은 노드와 container가 없는지 확인합니다.

```bash
ros2 node list
docker ps --filter label=com.xycar.role=integrated_drive
```

## 센서 점검

```bash
ros2 topic hz /image_raw
ros2 topic hz /scan
ros2 topic echo /camera_info --once
```

기준 자료에서는 카메라가 640×480 RGB8 약 30 Hz, LiDAR가 500 samples 약 9.66 Hz였습니다. 주행 전 frame id, 해상도, QoS와 주기를 함께 확인합니다.

## 분석 도구

- `tools/freshness_analysis`: 카메라 source timestamp부터 command까지의 freshness 확인
- `tools/yolo_optimization`: inference 환경, thread 수, FP16/CPU replay 비교
- `tools/full_drive_analysis`: 전체 주행 로그와 overlay 분석

도구별 인자는 `--help`와 각 디렉터리의 runbook을 우선 확인합니다. rosbag과 원본 영상은 용량과 개인정보 때문에 Git에서 제외되어 있습니다.

## 문제 확인 순서

1. `/image_raw`와 `/scan`의 주기·timestamp 확인
2. lane/scene detector 출력과 confidence 확인
3. mission mode와 pause/red latch 상태 확인
4. controller command freshness 확인
5. VESC handshake와 telemetry 확인
6. 명령이 0이라면 stale source, 미검출, pause 중 무엇이 원인인지 debug topic으로 분리

