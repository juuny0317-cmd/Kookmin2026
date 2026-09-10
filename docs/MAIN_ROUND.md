# Main Round — Real Vehicle Autonomous Driving

본선에서는 예선 simulator 중심 stack을 실제 Xycar Y 환경으로 확장했다. 전체 설계, 코드, 정량 검증과 실패 분석은 기존 위치를 유지했고 이 문서는 본선 자료의 landing page 역할을 한다.

> **전체 132팀 중 본선 22팀에 진출해 최종 7위를 기록했다.**

## Scope

| Area | Main Round implementation | Evidence |
|---|---|---|
| Camera perception | Lane/Scene Dual YOLO, OpenCV lane geometry, latest-frame routing | [Architecture](ARCHITECTURE.md), [`cam`](../ros2_ws/src/cam/) |
| LiDAR perception | DBSCAN clustering, obstacle/cone association | [Algorithms](ALGORITHMS.md), [`mission_cone_drive`](../ros2_ws/src/mission_cone_drive/) |
| Lane control | Speed/context-scheduled Stanley | [Algorithms](ALGORITHMS.md), [Engineering Iterations](ENGINEERING_ITERATIONS.md) |
| Cone control | Spline path + Pure Pursuit | [Algorithms](ALGORITHMS.md) |
| Mission decision | STOP/PAUSED/LANE/OVERTAKE/CONE arbitration | [Architecture](ARCHITECTURE.md) |
| Actuation & safety | Stamped command, freshness check, VESC adapter, watchdog | [Operations](OPERATIONS.md), [`xycar_motor_native`](../ros2_ws/src/xycar_motor_native/) |
| Sim-to-Real | 실측 geometry, asymmetric steering LUT, speed calibration, Gazebo | [Sim-to-Real](SIM_TO_REAL.md), [Calibration](REAL_VEHICLE_CALIBRATION.md) |

## Result

전체 132팀 중 예선을 통과한 22팀이 본선에 진출했고 Team SVE는 최종 7위를 기록했다. 최종 순위 화면에서 7위와 총 주행 시간 149.65초를 확인할 수 있으며, 참가·본선 진출 팀 규모는 사용자 제공 정보다.

<p align="center">
  <img src="../media/competition/final-ranking-7th.png" width="820" alt="Final competition ranking showing Team SVE in seventh place">
</p>

본선 2차 주행은 주행 144.65초와 penalty 5.00초를 합쳐 149.65초였다. 방송 카메라의 red indicator를 신호등으로 오인한 실패를 숨기지 않고 perception context gating과 validation coverage 문제로 분석했다. [경기 결과와 실패 분석](COMPETITION_RETROSPECTIVE.md)에서 확인할 수 있다.

## Competition Video

<p align="center">
  <img src="../media/video/competition_drive_2x.gif" width="820" alt="Team SVE main-round Xycar run at 2x speed">
</p>

<p align="center"><strong><a href="../media/video/competition_drive_2x.mp4">▶ 본선 연속 주행 MP4 재생</a></strong></p>

## Documentation Map

- [Root portfolio and Qualifying→Main journey](../README.md)
- [System Architecture](ARCHITECTURE.md)
- [Algorithms](ALGORITHMS.md)
- [Engineering Iterations](ENGINEERING_ITERATIONS.md)
- [Validation Evidence](VALIDATION_EVIDENCE.md)
- [Real Vehicle Calibration](REAL_VEHICLE_CALIBRATION.md)
- [Sim-to-Real](SIM_TO_REAL.md)
- [Operations](OPERATIONS.md)
- [Models](MODELS.md)
- [Sources & Licenses](SOURCE_AND_LICENSES.md)

예선 원본 source가 남아 있지 않다는 사실과 본선 코드의 출처 경계는 [예선 Source Availability](../qualifying_round/README.md#source-availability)와 [Sources & Licenses](SOURCE_AND_LICENSES.md)에 기록했다.
