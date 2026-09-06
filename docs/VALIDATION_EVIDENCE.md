# 인지 및 추종 검증 자료

이 문서는 공개 저장소에 포함한 인지 화면과 S자 구간 추종 그래프의 출처와 해석 범위를 기록합니다. 모든 이미지는 저장소의 체크포인트, 기록 프레임, CSV를 사용해 `tools/render_validation_evidence.py`로 다시 만들 수 있습니다. 수치와 입력 SHA-256은 `evaluation/validation_manifest.json`에 저장됩니다.

## 카메라 인지 파이프라인

검증 입력은 실차 카메라 기록에서 추출한 640×480 프레임입니다. 사람의 얼굴이나 위치 정보가 드러나지 않는 프레임을 골랐고 JPEG 메타데이터는 재인코딩 과정에서 제거했습니다.

| 단계 | 입력과 파라미터 | 기록된 결과 |
|---|---|---|
| Lane YOLO | `center_line_yolov10n_320_best.pt`, `imgsz=320`, confidence ≥ 0.25 | `center_line` 4개, 최고 confidence 0.924 |
| Scene YOLO | `all_second_yolo_v2_yolov10n_640_best_e46.pt`, `imgsz=640`, confidence ≥ 0.25 | `green` 1개, confidence 0.924 |
| OpenCV 후처리 | Lane YOLO ROI, Adaptive Gaussian Threshold(13, -20), Canny(50, 150), HoughLinesP(20, 20, 5) | ROI 4개, edge pixel 574개, Hough segment 9개 |

YOLO 출력은 중앙선 후보와 신호 상태를 분리해 보여 줍니다. OpenCV 화면의 주황색 사각형은 YOLO ROI, 청록색 픽셀은 Canny edge, 자홍색 선은 Hough line segment입니다. 이 순서는 `centerlane_tracer.py`에서 중앙선 곡선을 만들 때 사용하는 ROI 처리 단계와 같습니다.

## S자 구간 리플레이

`evaluation/s_curve/`의 CSV 두 개는 기록된 카메라 인지 결과를 재생하면서 기존 속도 정책 A와 10/12/16 3단 속도 정책 B를 비교한 결과입니다. 그래프 왼쪽은 이미지 중심에 대한 검출 차선 중심의 횡방향 오차, 오른쪽은 정책 B의 목표 속도와 변화율 제한이 적용된 출력 명령입니다.

| 기록 | 샘플 | 구간 | 명령–목표 MAE | 기존 정책의 위험한 10→16 점프 | 정책 B |
|---|---:|---:|---:|---:|---:|
| Run 02 | 589 | 39.46 s | 0.045 | 7 | 0 |
| Run 03 | 492 | 32.77 s | 0.028 | 3 | 0 |

정책 B는 곡선 이탈 뒤 0.5초 hold를 두어 두 기록 모두에서 검증되지 않은 10→16 직접 가속을 제거했습니다. 명령–목표 MAE는 가속 변화율 제한으로 생기는 짧은 지연을 포함합니다.

여기서 `cte_px`는 카메라 영상 평면에서 검출한 차선 중심과 영상 중심의 차이입니다. 곡선에서 차선이 화면 한쪽으로 이동하므로 차량의 실제 횡방향 위치 오차와 같지 않습니다. 또한 파란 선은 제어기가 발행할 속도 명령이며 엔코더로 측정한 바퀴 속도가 아닙니다. 따라서 이 자료는 인지 연속성과 명령 정책을 검증하지만 실제 궤적 오차나 속도 응답을 증명하지 않습니다.

## 재현

Ubuntu 22.04와 프로젝트의 Python 의존성이 설치된 환경에서 실행합니다.

```bash
python3 tools/render_validation_evidence.py
```

생성 파일은 다음과 같습니다.

- `media/perception/yolo_lane_scene.jpg`
- `media/perception/opencv_lane_roi.jpg`
- `media/validation/s_curve_tracking.png`
- `evaluation/validation_manifest.json`

원본 rosbag은 크기와 개인정보 문제로 Git 저장소에 포함하지 않습니다. 공개된 CSV, 선택 프레임, 체크포인트만으로 이 문서의 정적 결과를 다시 만들 수 있습니다.
