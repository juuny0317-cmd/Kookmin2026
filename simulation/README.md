# 국민대 DXF Gazebo 월드

`국민대_트랙_도면스케일_mm_10배.dxf`의 닫힌 폴리라인 125개를 분석해 만든 Gazebo Sim 월드이다. 도면의 곡선(bulge), 외곽 형상, 점선 및 내부 섬 형상을 유지하고, 여섯 개의 기존 도로 경계선만 아래의 요청 규격에 맞는 오프셋 경계선으로 교체했다. 도면에 없는 주차선이나 장애물은 추가하지 않는다.

곡선부 점선 폴리곤 일부가 Gazebo 삼각분할 과정에서 생략되는 문제를 방지하기 위해, 노란색 중앙 점선은 DXF에 기록된 모든 중심 위치와 진행 방향을 사용하면서 `길이 0.20 m × 폭 0.05 m`의 동일한 형상으로 렌더링한다.

도로 횡단면은 노란 중앙선을 유지하면서 다음 규격으로 렌더링한다.

- 회색 도로 폭: `0.80 m`
- 흰색 외곽선: 회색 도로 바깥 양쪽에 각각 `0.05 m`
- 흰색 외곽부터 반대편 흰색 외곽까지 총 폭: `0.90 m`

## 크기와 좌표

- DXF R10 파일 자체에는 단위 메타데이터가 없으므로 파일명에 표시된 `mm`를 적용한다.
- 기본 변환은 `1 DXF unit = 1 mm = 0.001 m`이며 임의의 폭 맞춤 보정은 하지 않는다.
- 도면의 +X/+Y 방향을 그대로 유지하고 외곽 형상의 중심만 Gazebo 원점 `(0, 0)`으로 이동한다.
- 정확한 최종 외곽 크기는 생성된 SDF 파일 상단 주석에도 기록된다.
- 노면 표시에는 충돌을 넣지 않았고 전체 맵은 평평한 단일 충돌면을 사용한다.

## 실행

저장소 루트에서 다음 명령을 실행한다.

```bash
chmod +x scripts/run_kookmin_dxf_world.sh
./scripts/run_kookmin_dxf_world.sh
```

또는 Gazebo 명령을 직접 실행할 수 있다.

```bash
gz sim -r "$(pwd)/worlds/kookmin_track_from_dxf.world.sdf"
```

## DXF에서 월드 재생성

원본 DXF는 저장소에 포함하지 않는다. 정식으로 제공받은 도면이 있을 때 경로를 직접 지정한다.

```bash
python3 scripts/generate_dxf_course_world.py \
  "/absolute/path/to/kookmin_track.dxf" \
  worlds/kookmin_track_from_dxf.world.sdf
```

DXF 단위를 다르게 해석해야 할 때만 `--meters-per-unit` 값을 변경한다. 예를 들어 한 DXF 단위가 1 cm라면 `--meters-per-unit 0.01`을 사용한다.

## 차량 시작 위치 예시

월드 이름은 `kookmin_dxf_track`이다. 시작점은 하단 직선의 중앙 차선 부근인 다음 좌표를 권장한다.

```text
x = 0.0 m, y = -3.55 m, z = 차량에 맞는 높이, yaw = 0 rad
```
