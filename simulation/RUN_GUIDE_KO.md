# K-City Gazebo 맵 실행 안내

## 포함 내용

- `worlds/`: Gazebo 월드 파일
- `models/`: 도로 및 교차로 모델
- `xycar_gz_sim/`: ROS 2 차량·센서 시뮬레이터
- `team_code/track_drive/`: 카메라 차선주행 코드
- `scripts/`: 맵 단독 실행 및 DXF 변환 스크립트

## 권장 환경

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Sim Harmonic (`gz sim` 8.x)
- `colcon`, `rosdep`

아래 명령으로 이미 설치된 버전을 확인할 수 있다.

```bash
source /opt/ros/humble/setup.bash
ros2 --help >/dev/null && echo "ROS 2 확인 완료"
gz sim --versions
```

## 1. 압축 풀기 및 의존성 설치

압축 파일을 `~/Competitions/2026/00_Shared_Multi_Contest/planning_ws/src` 아래에 푼다.

```bash
mkdir -p ~/Competitions/2026/00_Shared_Multi_Contest/planning_ws/src
cd ~/Competitions/2026/00_Shared_Multi_Contest/planning_ws/src
unzip kcity_gazebo_share_20260806.zip
cd ~/Competitions/2026/00_Shared_Multi_Contest/planning_ws

source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

`rosdep` 실행 중 필요한 시스템 패키지 설치를 위해 비밀번호를 물을 수 있다.

## 2. 빌드

```bash
cd ~/Competitions/2026/00_Shared_Multi_Contest/planning_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select track_drive xycar_gz_sim
source install/setup.bash
```

새 터미널을 열 때마다 다음 두 줄은 다시 실행한다.

```bash
source /opt/ros/humble/setup.bash
source ~/Competitions/2026/00_Shared_Multi_Contest/planning_ws/install/setup.bash
```

## 3. 실행 방법

### 맵만 실행

ROS 차량 없이 Gazebo 맵만 확인하려면 다음을 실행한다.

```bash
cd ~/Competitions/2026/00_Shared_Multi_Contest/planning_ws/src/kcity_gazebo
chmod +x scripts/run_kookmin_dxf_world.sh
./scripts/run_kookmin_dxf_world.sh
```

### 차량과 센서 실행

```bash
source /opt/ros/humble/setup.bash
source ~/Competitions/2026/00_Shared_Multi_Contest/planning_ws/install/setup.bash
ros2 launch xycar_gz_sim xycar.launch.py use_sim:=true gui:=true
```

차량을 키보드로 운전하려면 두 번째 터미널에서 실행한다.

```bash
source /opt/ros/humble/setup.bash
source ~/Competitions/2026/00_Shared_Multi_Contest/planning_ws/install/setup.bash
ros2 run xycar_gz_sim keyboard_teleop
```

키는 `W/S` 전진·감속, `A/D` 좌·우 조향, `Space` 정지, `C` 조향 중앙,
`X` 정지 및 중앙 복귀, `Q` 종료다.

### 팀 차선주행까지 한 번에 실행

```bash
source /opt/ros/humble/setup.bash
source ~/Competitions/2026/00_Shared_Multi_Contest/planning_ws/install/setup.bash
ros2 launch xycar_gz_sim team_lane_sim.launch.py gui:=true
```

차량과 센서만 먼저 확인하고 자동주행은 시작하지 않으려면 다음을 사용한다.

```bash
ros2 launch xycar_gz_sim team_lane_sim.launch.py start_drive:=false gui:=true
```

종료할 때는 launch를 실행한 터미널에서 `Ctrl-C`를 한 번 누른다.

## 문제 해결

- `Package 'xycar_gz_sim' not found`: 현재 터미널에서
  `source ~/Competitions/2026/00_Shared_Multi_Contest/planning_ws/install/setup.bash`를 실행한다.
- 월드를 찾을 수 없음: 압축을 `~/Competitions/2026/00_Shared_Multi_Contest/planning_ws/src/kcity_gazebo` 구조로 풀었는지 확인하고
  두 패키지를 다시 빌드한다.
- Gazebo가 중복 실행되었다는 오류: 다른 시뮬레이션 창을 정상 종료한 뒤 다시 실행한다.
- GUI가 느리거나 없는 PC: launch 뒤에 `gui:=false`를 지정한다.
- 상세 토픽, 보정값, 검증 방법은 `xycar_gz_sim/README.md`를 참고한다.
