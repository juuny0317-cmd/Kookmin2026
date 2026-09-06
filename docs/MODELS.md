# 모델 체크포인트

최종 통합 실행에서 직접 선택되는 파일은 다음 두 개입니다.

| 역할 | 파일 | 입력 | SHA-256 |
|---|---|---:|---|
| 차선 중심선 | `center_line_yolov10n_320_best.pt` | 320 | `3bf397fdb5b6adf13da9f58664fbf723261dbf8b3246c557e3880cdbee21fc3d` |
| 신호·콘·장애물 | `all_second_yolo_v2_yolov10n_640_best_e46.pt` | 640 | `80fa0a345164211832fdcc9af4cab6e44fddbf1dbcb8ddcaec296b31f3d9c0b5` |

최종 후보 비교와 회귀 확인에 사용한 대회용 체크포인트도 함께 보존했습니다.

| 파일 | SHA-256 |
|---|---|
| `all_second_yolo_yolov10n_640_best_e36_deploy.pt` | `d229954c1f15390bbdd093add09bd50548d86ed4bb503dff405abe78ec80326d` |
| `xycar_yolov10n_5class_best.pt` | `29581c25366d048032fae4d79c65b04f298af098685b9d5b0bced68d5862b1b6` |
| `xycar_yolov10n_5class_cone_v1.pt` | `4d42f46590abec4cd7a7ca8f195539b7961bc864227a71bd06d774ed44858a8b` |
| `xycar_yolov10n_5class_cone_v2.pt` | `17539bb37729af59139566a1c4ab6a8199b8c4a9e592118a26498d1ea15031da` |
| `xycar_yolov10n_5class_cone_v3.pt` | `bc45b60651620bee67569f448e975bc73d200b5ed2ab4bd02792b79d60a895dc` |
| `xycar_yolov10n_5class_cone_v4.pt` | `ac9d217e09f8c7855df3b24286f5cdc4eeabc4caaeaa3e6fbc1ad3e3846e86f6` |
| `yolov10n_17000_1.pt` | `cdf59a7c216091a540985ff2c3e1a93b728c029016f8c24de82a753ccef5f20e` |
| `yolov10n_20000.pt` | `dd2f5e40dba630daede6bcf12d611327de9d42bc1206db2d7c518016748ab55b` |

모든 파일은 `ros2_ws/src/cam/cam/`에 있습니다. 저장소 용량을 줄이기 위해 이 시스템과 관계없는 YOLOv8 medium detection/pose/segmentation 데모 가중치는 제외했습니다.

체크포인트는 실행 재현을 위한 산출물입니다. 학습 데이터셋은 개인정보·용량·배포 범위 문제로 포함하지 않습니다.

