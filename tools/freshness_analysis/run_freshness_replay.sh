#!/usr/bin/env bash
set -eo pipefail

if [[ $# -lt 1 || $# -gt 2 ]]; then
  echo "usage: $0 RUN_DIR [off|profile]" >&2
  exit 2
fi

RUN_DIR=$1
PROFILE_MODE=${2:-off}
WORKSPACE=/home/xytron/xycar_ws
INPUT_BAG=/home/xytron/Downloads/oscillation_20260806_104536_replay
MOTOR_TOPIC=/offline/freshness_test_xycar_motor

if [[ "$PROFILE_MODE" != off && "$PROFILE_MODE" != profile ]]; then
  echo "profile mode must be off or profile" >&2
  exit 2
fi
if [[ -e "$RUN_DIR" ]]; then
  echo "refusing to reuse existing run directory: $RUN_DIR" >&2
  exit 2
fi
mkdir -p "$RUN_DIR"

source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"
set -u
export ROS_DOMAIN_ID=79
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

PROFILE_ENABLED=false
if [[ "$PROFILE_MODE" == profile ]]; then
  PROFILE_ENABLED=true
fi

launch_pid=''
record_pid=''
probe_pid=''
cleanup_done=false
cleanup() {
  if [[ "$cleanup_done" == true ]]; then
    return
  fi
  cleanup_done=true
  if [[ -n "$record_pid" ]] && kill -0 "$record_pid" 2>/dev/null; then
    kill -INT "$record_pid" 2>/dev/null || true
    wait "$record_pid" 2>/dev/null || true
  fi
  if [[ -n "$probe_pid" ]] && kill -0 "$probe_pid" 2>/dev/null; then
    kill -TERM "$probe_pid" 2>/dev/null || true
    wait "$probe_pid" 2>/dev/null || true
  fi
  if [[ -n "$launch_pid" ]] && kill -0 "$launch_pid" 2>/dev/null; then
    pkill -TERM -P "$launch_pid" 2>/dev/null || true
    kill -TERM "$launch_pid" 2>/dev/null || true
    wait "$launch_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

printf '%s\n' \
  "kind=offline_replay" \
  "profile_mode=$PROFILE_MODE" \
	  "input_bag=$INPUT_BAG" \
	  "motor_topic=$MOTOR_TOPIC" \
	  "timeout_s=0.30" \
  "created=$(date --iso-8601=seconds)" \
  >"$RUN_DIR/run_manifest.txt"

ros2 launch mission_cone_drive integrated_drive.launch.py \
  start_camera:=false start_lidar:=false start_ultrasonic:=false \
  start_yolo:=true start_lane_stack:=true start_rviz:=false \
  start_active:=false use_sim_time:=true \
  enable_lane_pipeline:=true enable_scene_pipeline:=false \
  enable_drive_control:=true \
  lane_model_filename:=center_line_yolov10n_320_best.pt \
  lane_image_width:=320 lane_image_height:=240 \
  lane_max_rate_hz:=12.0 lane_yolo_max_rate_hz:=0.0 \
  lane_inference_image_size:=320 lane_class_names:=center_line \
  lane_torch_compile_mode:=none lane_torch_threads:=4 \
  lane_torch_interop_threads:=1 lane_opencv_threads:=1 \
  lane_cpu_affinity:=0-7 lane_yolo_device:=cpu lane_yolo_fp16:=false \
  camera_qos_reliability:=reliable \
  lane_image_qos_reliability:=reliable \
	  lane_straight_speed:=4.0 lane_curve_strong_speed:=4.0 \
	  lane_curve_medium_speed:=4.0 lane_curve_mild_speed:=4.0 \
	  lane_straight_recovery_speed:=4.0 \
	  lane_curve_decel_rate:=96.0 \
	  lane_straight_decel_rate:=36.0 \
  stanley_control_rate_hz:=20.0 mission_decision_rate_hz:=20.0 \
  stanley_state_timeout_s:=0.30 \
  frame_router_event_driven:=true stanley_event_driven:=true \
  mission_event_driven_lane:=false \
  lane_lateral_lane_half_width_px:=250.0 \
  lane_lateral_outer_pair_tolerance_px:=80.0 \
  lane_lateral_outer_pair_hold_frames:=5 \
  show_lane_view:=false show_lane_yolo_view:=false \
  show_lane_processing_view:=false \
  publish_pipeline_timing:="$PROFILE_ENABLED" \
  publish_performance_stats:=false \
  publish_resource_stats:="$PROFILE_ENABLED" \
  resource_stats_rate_hz:=2.0 \
  resource_stats_output_path:="$RUN_DIR/resource_stats.csv" \
  resource_stats_process_pattern:=lane_yolo_node \
  resource_stats_profile_threads:=false \
  publish_camera_timing:=false \
  motor_topic:="$MOTOR_TOPIC" \
  >"$RUN_DIR/launch.log" 2>&1 &
launch_pid=$!

for _ in $(seq 1 90); do
  if ros2 service list 2>/dev/null | grep -qx '/start_integrated_drive'; then
    break
  fi
  sleep 1
done
if ! ros2 service list 2>/dev/null | grep -qx '/start_integrated_drive'; then
  echo 'start service did not appear' >&2
  exit 3
fi

if [[ "$PROFILE_ENABLED" == true ]]; then
  python3 "$WORKSPACE/src/freshness_analysis/freshness_probe.py" \
    --ros-args \
    -p use_sim_time:=true \
    -p output_path:="$RUN_DIR/freshness_probe.csv" \
    -p final_motor_topic:="$MOTOR_TOPIC" \
    -p observe_images:=true \
    -p observe_pipeline_timing:=true \
    >"$RUN_DIR/probe.log" 2>&1 &
  probe_pid=$!
  for _ in $(seq 1 30); do
    if ros2 node list 2>/dev/null | grep -qx '/freshness_probe'; then
      break
    fi
    sleep 0.2
  done
  ros2 service call /resource_profiler/refresh_processes \
    std_srvs/srv/Trigger '{}' \
    >"$RUN_DIR/resource_refresh.log" 2>&1 || true
fi

ros2 bag record --storage sqlite3 --storage-preset-profile resilient \
  --max-cache-size 104857600 --use-sim-time \
  -o "$RUN_DIR/replay_output_bag" \
  /clock /lane_yolo/detections /center_curve \
  /lane_control_state_stamped /lane_control_state_v2 \
  /xycar_state_stamped /stanley/debug /stanley/debug_v2 \
  /lane_motor_cmd_stamped "$MOTOR_TOPIC" \
  /mission_status /mission_mode /pipeline_timing \
  >"$RUN_DIR/record.log" 2>&1 &
record_pid=$!

sleep 2
ros2 service call /start_integrated_drive std_srvs/srv/Trigger '{}' \
  >"$RUN_DIR/service.log" 2>&1
sleep 2

ros2 bag play "$INPUT_BAG" \
  --storage sqlite3 --clock 30 --rate 1.0 \
  --read-ahead-queue-size 1000 --wait-for-all-acked 1000 \
  --disable-keyboard-controls \
  --topics /image_raw /camera_info \
  >"$RUN_DIR/play.log" 2>&1

sleep 5
cleanup
trap - EXIT INT TERM
printf 'completed=%s\n' "$(date --iso-8601=seconds)" >"$RUN_DIR/run_complete.txt"
echo "motor-safe freshness replay completed: $RUN_DIR"
