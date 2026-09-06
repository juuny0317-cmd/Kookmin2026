# Evaluation data

공개 검증 그림을 재현하는 데 필요한 최소 입력만 보존합니다.

- `perception/frame_0490_t231582ms.jpg`: 실차 카메라 기록에서 선별하고 메타데이터를 제거한 프레임
- `s_curve/s_curve_run02_ab.csv`, `s_curve/s_curve_run03_ab.csv`: S자 구간 인지 리플레이 결과
- `s_curve/*_summary.json`: 기존 정책과 3단 속도 정책 비교 요약
- `validation_manifest.json`: 입력·모델 SHA-256, 검출 결과, 그래프 통계

대용량 rosbag, 학습 데이터 전체, 얼굴이 식별되는 프레임은 포함하지 않습니다. 산출물 재생성 방법과 해석 범위는 [`docs/VALIDATION_EVIDENCE.md`](../docs/VALIDATION_EVIDENCE.md)에 있습니다.
