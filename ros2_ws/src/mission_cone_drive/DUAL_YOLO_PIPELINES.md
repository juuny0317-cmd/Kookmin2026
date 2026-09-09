# Dual YOLO perception pipelines

`integrated_drive.launch.py`는 하나의 camera stream을 lane과 scene branch로 나눈다. `centerlane_tracer`와 `Lane_Detector`의 Canny/Hough, lane role, heading/curvature 계산은 router나 YOLO node에 합쳐지지 않는다.

## Competition / final launch

Source of truth: `scripts/start_integrated_drive_container.sh`

```text
/image_raw (640x480)
  -> frame_router
     -> /perception/lane/image  (320x240, max 15 Hz)
        -> lane_yolo_node [center_line_yolov10n_320_best.pt, imgsz 320]
           -> /lane_yolo/detections
              -> centerlane_tracer -> /center_curve/base or /center_curve
                 -> shortcut_left_turn_node -> /center_curve
                    -> lane_detector -> /lane_control_state_v3
                       -> integrated_stanley_controller
                          -> /lane_motor_cmd_stamped
                             -> mission_manager_node -> /xycar_motor
     -> /perception/scene/image (640x480, max 10 Hz)
        -> scene_yolo_node
           [all_second_yolo_v2_yolov10n_640_best_e46.pt, imgsz 640]
           -> /scene_yolo/detections
```

## Code default와의 차이

| Setting | Code default | Competition / final launch |
|---|---:|---:|
| lane router | 12 Hz | **15 Hz** |
| scene router | compatibility default 4 Hz | **10 Hz** |
| lane imgsz | 320 | **320** |
| scene imgsz | 640 | **640** |
| event-driven latest-frame routing | false | **true** |

`frame_router.py`의 12 Hz와 4 Hz는 isolated A/B 시험을 보존하기 위한 node default다. `tools/freshness_analysis/*.sh`와 `mission_cone_drive/scripts/integrated_drive_with_bridge`의 12 Hz도 특정 replay/bridge preset이며 최종 대회 launcher가 아니다.

## Detection consumers

| Consumer | Lane detections | Scene detections | Notes |
|---|---|---|---|
| `centerlane_tracer` | required | cached, optional | Scene box는 lane 좌표로 scale하고 stale이면 버린다. |
| `Lane_Detector` | required | cached, optional | Scene stream이 lane processing cadence를 throttle하지 않는다. |
| `traffic_light_node` | no | required | red/green/left direct class를 읽는다. |
| `target_lane_planner` | no | required | dynamic/static obstacle과 LiDAR cluster를 연결한다. |
| `cone_entry_fusion_node` | no | required | cone box와 cluster를 source-stamp gate로 결합한다. |
| `mission_manager_node` | no | optional | scene/cone mission이 켜졌을 때만 구독한다. |

두 image output은 keep-last depth 1로 동작하고 원본 camera header를 보존한다. worker가 바쁜 동안 새 프레임이 오면 대기 중인 오래된 프레임을 최신 프레임으로 교체하므로 FIFO backlog가 누적되지 않는다.

## Modes

```bash
# Lane only
ros2 launch mission_cone_drive integrated_drive.launch.py \
  enable_lane_pipeline:=true enable_scene_pipeline:=false \
  enable_drive_control:=true start_active:=false

# Scene only
ros2 launch mission_cone_drive integrated_drive.launch.py \
  enable_lane_pipeline:=false enable_scene_pipeline:=true \
  enable_drive_control:=false

# Full stack
ros2 launch mission_cone_drive integrated_drive.launch.py \
  enable_lane_pipeline:=true enable_scene_pipeline:=true \
  enable_drive_control:=true start_active:=false
```

Rosbag replay에서는 `start_camera:=false use_sim_time:=true`를 사용한다. 저장된 일부 bag은 camera header가 bag clock보다 약 1.1 s 느리므로, offline 분석용 timeout 조정 없이 motor controller를 활성화하지 않는다.
