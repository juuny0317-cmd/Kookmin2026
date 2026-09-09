# square_intersection_gazebo_map

Gazebo / `gz sim`용 중앙 십자 교차로 + 정사각형 외곽 순환도로 맵이다.

이번 버전은 중앙 교차로 주변 4개의 직선 구간에 참고 이미지와 같은 상세 노면 마킹을 넣었다.

## 치수

- 외곽 순환도로 중심선 기준 한 변: 100.0 m
- 중심선 좌표 범위: x/y = -50 m ~ +50 m
- 도로 폭: 7.0 m
- 한 차선 폭: 3.5 m
- 외곽 순환도로 모서리 중심선 반지름: 12.0 m
- 중앙 교차로 중심: x=0, y=0
- 상세 직선 구간 길이 기준: 48.0 m

## 중앙 교차로 주변 4개 직선구간 마킹

각 방향 직선구간에 동일하게 적용했다.

- 교차로 쪽 끝 횡단보도
- 흰색 마름모 예고 마킹 4개
- 차선별 진행방향 화살표
- 이중 노란 중앙 실선
- 양쪽 흰색 가장자리 실선

## 충돌 설정

- 아스팔트: collision 있음
- 노란 중앙선 / 흰 실선 / 횡단보도 / 마름모 / 화살표 / 정지선: visual-only, collision 없음
- 노면표시는 z-fighting 방지를 위해 아스팔트보다 0.5~1.0 mm 위에 있다.
- 노면표시에는 collision이 없으므로 차량이 선이나 마킹을 밟아도 덜컹거리지 않는다.

## 설치

```bash
mkdir -p ~/Competitions/2026/03_Kookmin_Autonomous_9th/gazebo_maps
cd ~/Competitions/2026/03_Kookmin_Autonomous_9th/gazebo_maps
unzip ~/Downloads/square_intersection_gazebo_map.zip -d ~/Competitions/2026/03_Kookmin_Autonomous_9th/gazebo_maps
```

기존 폴더가 있으면 먼저 삭제해야 한다.

```bash
rm -rf ~/Competitions/2026/03_Kookmin_Autonomous_9th/gazebo_maps/square_intersection_gazebo_map
unzip ~/Downloads/square_intersection_gazebo_map.zip -d ~/Competitions/2026/03_Kookmin_Autonomous_9th/gazebo_maps
```

## 실행

```bash
export GZ_SIM_RESOURCE_PATH=$HOME/gazebo_maps:$GZ_SIM_RESOURCE_PATH
gz sim -r ~/Competitions/2026/03_Kookmin_Autonomous_9th/gazebo_maps/square_intersection_gazebo_map/worlds/square_intersection_loop.world.sdf
```

## 차량 spawn 예시

```bash
ros2 run ros_gz_sim create \
  -world square_intersection_loop \
  -file /path/to/your_vehicle.sdf \
  -name ego_vehicle \
  -x 0.0 -y -50.0 -z 0.30 -Y 1.5708
```

## 재생성

```bash
cd ~/Competitions/2026/03_Kookmin_Autonomous_9th/gazebo_maps/square_intersection_gazebo_map
python3 scripts/generate_square_intersection_map.py
```
