# 검증 기록

2026-09-07, Ubuntu 22.04 / ROS 2 Humble 환경에서 공개 저장소 구성을 검증했다.

| 검사 | 결과 |
|---|---|
| 인지·판단·제어·구동 변환·시뮬레이션 단위 테스트 | 283 passed |
| Python source compile | passed |
| 통합·bridge shell script syntax | passed |
| `track_drive` + `xycar_gz_sim` colcon build | passed, 2 packages |
| ROS 메시지·카메라·미션·LiDAR 패키지 build | passed |
| VESC driver build | host dependency `ros-humble-serial-driver` 필요 |
| GitHub 파일 크기 확인 | 100 MB 초과 파일 없음 |
| source tree symlink 확인 | 없음 |

현재 PC에는 sudo 비밀번호 없이 system dependency를 추가할 수 없어 VESC driver의 마지막 링크까지 실행하지 않았다. `vesc_driver/package.xml`에는 `serial_driver`가 선언되어 있으며, 실제 차량 장비에서는 빌드 전에 다음 의존성을 설치해야 한다.

```bash
sudo apt install \
  ros-humble-ackermann-msgs \
  ros-humble-serial-driver \
  python3-sklearn
```

IMU 패키지까지 사용할 경우 Python `transforms3d`도 필요하다. 실제 차량 주행은 하드웨어 연결, 장착 pose와 emergency stop 조건이 필요하므로 이 저장소 정리 과정에서 수행하지 않았다.

