# Dual YOLO perception pipelines

`integrated_drive.launch.py` routes one camera stream into independent lane and
scene branches. `centerlane_tracer` and `Lane_Detector` remain separate nodes;
their Canny/Hough, `self.lane`, `self.angle`, center-line correction, and Stanley
logic are not merged into the router or either YOLO node.

```text
/image_raw (640x480)
  -> frame_router
     -> /perception/lane/image  (320x240 @ 12 Hz)
        -> lane_yolo_node [center_line_yolov10n_320_best.pt, center_line]
           -> /lane_yolo/detections
              -> centerlane_tracer -> /center_curve
                 -> lane_detector -> /xycar_state(_stamped)
                    -> integrated_stanley_controller
                       -> /lane_motor_cmd(_stamped)
                          -> mission_manager_node -> /xycar_motor
     -> /perception/scene/image (640x480 @ 4 Hz)
        -> scene_yolo_node [5-class cone v4 model]
           -> /scene_yolo/detections
```

## Detection consumers

| Consumer | Lane detections | Scene detections | Notes |
|---|---|---|---|
| `centerlane_tracer` | required | cached, optional | Scene boxes are scaled into lane-image coordinates and rejected when stale. |
| `Lane_Detector` | required | cached, optional | Scene data is not part of the 12 Hz synchronizer. |
| `traffic_light_node` | no | required | Reads `race_traffic_light`. |
| `target_lane_planner` | no | required | Uses obstacle detections; curve coordinates are scaled separately. |
| `cone_entry_fusion_node` | no | required | Reads cone detections with source-stamp freshness checks. |
| `Integrated_Stanley_Controller` | no | optional | Traffic/obstacle inputs are absent in lane-only mode. |
| `mission_manager_node` | no | optional | Scene and cone subscriptions are absent in lane-only mode. |

Both detection topics use reliable QoS. Large image outputs use keep-last depth
1 and preserve the original camera Header. Disabled branches have no publisher,
resize timer, model load, or inference worker.

## Modes

After sourcing the ROS and workspace setup files:

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

# Camera only (both perception branches off)
ros2 launch mission_cone_drive integrated_drive.launch.py \
  enable_lane_pipeline:=false enable_scene_pipeline:=false \
  enable_drive_control:=false
```

The default lane model is compiled and warmed before it subscribes. The scene
model is eagerly warmed and runs at a lower OS scheduling priority so lane state
remains the priority under CPU pressure. These controls can be changed with
`lane_torch_compile_mode`, `scene_torch_compile_mode`, `lane_process_nice`, and
`scene_process_nice`. Use `publish_performance_stats:=true` for periodic metrics;
leave all visualization flags false while measuring performance.

For rosbag replay, add `start_camera:=false use_sim_time:=true` and play the bag
with `--clock`. The recorded bags currently have camera Header stamps about
1.1 seconds behind bag clock, so do not enable the motor controller during a
performance replay unless its freshness timeout is adjusted specifically for
that offline test.
