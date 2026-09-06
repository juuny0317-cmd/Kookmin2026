# 대회 요구사항 정리

이 문서는 국민대학교 제9회 자율주행 경진대회 2026년 경주 규정을 구현 관점에서 직접 요약한 것입니다. 규정 원문에는 무단 전재·재배포 금지 표시가 있어 PDF 자체와 도면 원본은 이 저장소에 포함하지 않습니다. 실제 참가와 판정에는 주최 측이 배포한 최신 원문을 기준으로 해야 합니다.

## 본선 주행 흐름

1. 신호 인식 후 출발
2. 콘 구간 주행
3. 차선 주행
4. 정적 장애물 회피
5. 주행 중인 간섭 차량 인지와 추월
6. 총 3바퀴 주행 중 지정된 한 바퀴에서 지름길 수행
7. 결승선 통과

지름길은 두 번째 또는 세 번째 바퀴에 제시되는 좌회전 화살표/녹색 신호에 따라 진입할 수 있습니다. 별도의 주차 종목도 존재합니다.

## 코드 대응

| 요구 기능 | 구현 위치 |
|---|---|
| 신호 출발/정지 | scene YOLO + traffic direct decision + red latch |
| 콘 구간 | LiDAR DBSCAN + camera fusion + spline path + Pure Pursuit |
| 차선 주행 | lane YOLO + BEV/Hough + Stanley |
| 정적 장애물 | static class + LiDAR 거리 + 반대 차선 timed event |
| 동적 차량 | dynamic class + temporal tracking + 추월 event |
| 지름길 | left/green signal + shortcut staged state machine |
| 안전 정지 | pause, freshness, VESC readiness, stale command zeroing |

## 저장소에 포함하지 않은 자료

- 대회 규정 PDF 원문
- 대회 제공 DXF 원본
- 전체 실차 영상과 rosbag
- 참가자 또는 팀원의 개인정보가 포함된 자료

대신 코드에서 검증할 수 있는 실행 파라미터, 생성된 Gazebo world, 보정 설정과 식별 정보가 제거된 대표 이미지를 제공합니다.

