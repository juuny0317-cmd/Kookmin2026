#!/usr/bin/env bash
# Run one motor-isolated C1 replay. Invoke once per thread configuration.

set -euo pipefail

usage() {
  echo "Usage: $0 BAG MODEL OUTPUT_ROOT RUN_NAME INTRA INTEROP [PROFILER] [AFFINITY] [PROFILE_THREADS] [CLOCK_HZ] [INPUT_MODE] [RESOURCE_HZ]"
  echo "  PROFILER: on|off; AFFINITY: taskset list; PROFILE_THREADS: true|false; CLOCK_HZ: default 100"
  echo "  INPUT_MODE: raw (contention/cadence) or fixed_lane (source-aligned regression)"
}

if [ "$#" -lt 6 ] || [ "$#" -gt 12 ]; then
  usage
  exit 2
fi

BAG_PATH=$(realpath "$1")
MODEL_PATH=$(realpath "$2")
OUTPUT_ROOT=$(realpath -m "$3")
RUN_NAME=$4
INTRA_THREADS=$5
INTEROP_THREADS=$6
PROFILER_MODE=${7:-on}
CPU_AFFINITY=${8:-0-7}
PROFILE_THREADS=${9:-false}
CLOCK_HZ=${10:-100}
INPUT_MODE=${11:-raw}
RESOURCE_HZ=${12:-2.0}

if [ ! -d "$BAG_PATH" ] || [ ! -f "$MODEL_PATH" ]; then
  echo "Bag or model path does not exist."
  exit 2
fi
if ! [[ "$INTRA_THREADS" =~ ^[1-9][0-9]*$ ]] || \
   ! [[ "$INTEROP_THREADS" =~ ^[1-9][0-9]*$ ]]; then
  echo "Thread counts must be positive integers."
  exit 2
fi
if [ "$PROFILER_MODE" != on ] && [ "$PROFILER_MODE" != off ]; then
  echo "PROFILER must be on or off."
  exit 2
fi
if [ "$PROFILE_THREADS" != true ] && [ "$PROFILE_THREADS" != false ]; then
  echo "PROFILE_THREADS must be true or false."
  exit 2
fi
if ! [[ "$CLOCK_HZ" =~ ^[1-9][0-9]*$ ]]; then
  echo "CLOCK_HZ must be a positive integer."
  exit 2
fi
if [ "$INPUT_MODE" != raw ] && [ "$INPUT_MODE" != fixed_lane ]; then
  echo "INPUT_MODE must be raw or fixed_lane."
  exit 2
fi
if ! [[ "$RESOURCE_HZ" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "RESOURCE_HZ must be a non-negative number."
  exit 2
fi
if ! [[ "$RUN_NAME" =~ ^[A-Za-z0-9_.-]+$ ]]; then
  echo "RUN_NAME may contain only letters, digits, dot, underscore and dash."
  exit 2
fi

RUN_DIR="$OUTPUT_ROOT/$RUN_NAME"
OUTPUT_BAG="$RUN_DIR/bag"
RUNTIME_JSON="$RUN_DIR/yolo_runtime.json"
RESOURCE_CSV="$RUN_DIR/resources.csv"
if [ -e "$RUN_DIR" ]; then
  echo "Refusing to overwrite existing run directory: $RUN_DIR"
  exit 3
fi
mkdir -p "$RUN_DIR"

for _ in $(seq 1 60); do
  if ! ros2 node list 2>/dev/null | grep -Eq \
      '^/(lane_yolo_node|frame_router|integrated_stanley_controller)$'; then
    break
  fi
  sleep 0.5
done
if ros2 node list 2>/dev/null | grep -Eq \
    '^/(lane_yolo_node|frame_router|integrated_stanley_controller)$'; then
  echo "Existing drive-stack nodes detected; stop them before replay."
  exit 4
fi

LAUNCH_PID=
RECORDER_PID=
PLAYER_PID=
terminate_process() {
  local pid=$1
  local signal=${2:-TERM}
  if ! kill -0 "$pid" 2>/dev/null; then
    return
  fi
  kill -"$signal" "$pid" 2>/dev/null || true
  for _ in $(seq 1 50); do
    if ! kill -0 "$pid" 2>/dev/null; then
      wait "$pid" 2>/dev/null || true
      return
    fi
    sleep 0.1
  done
  kill -KILL "$pid" 2>/dev/null || true
  wait "$pid" 2>/dev/null || true
}
cleanup() {
  set +e
  if [ -n "$PLAYER_PID" ]; then
    terminate_process "$PLAYER_PID" TERM
  fi
  if [ -n "$RECORDER_PID" ]; then
    terminate_process "$RECORDER_PID" INT
  fi
  if [ -n "$LAUNCH_PID" ]; then
    terminate_process "$LAUNCH_PID" TERM
  fi
}
trap cleanup EXIT INT TERM

RESOURCE_ENABLED=false
if [ "$PROFILER_MODE" = on ]; then
  RESOURCE_ENABLED=true
fi
SCENE_PIPELINE_ENABLED=true
PLAY_TOPICS=(/image_raw)
if [ "$INPUT_MODE" = fixed_lane ]; then
  # Replay the 114 routed C1 frames themselves.  This deliberately removes
  # frame-router sampling and scene-YOLO contention so two configurations can
  # be compared on exactly the same source bytes.  Use raw mode for cadence
  # and resource selection; fixed_lane is only the regression gate.
  SCENE_PIPELINE_ENABLED=false
  PLAY_TOPICS=(/perception/lane/image)
fi

ros2 launch mission_cone_drive integrated_drive.launch.py \
  start_camera:=false \
  start_lidar:=false \
  start_ultrasonic:=false \
  start_yolo:=true \
  start_lane_stack:=true \
  start_rviz:=false \
  start_active:=false \
  use_sim_time:=true \
  motor_topic:=/cpu_ab/motor \
  show_yolo_view:=false \
  show_lane_view:=false \
  show_overtake_view:=false \
  enable_lane_pipeline:=true \
  enable_scene_pipeline:="$SCENE_PIPELINE_ENABLED" \
  enable_drive_control:=true \
  frame_router_event_driven:=true \
  stanley_event_driven:=true \
  mission_event_driven_lane:=false \
  stanley_state_timeout_s:=0.30 \
  lane_model_filename:="$MODEL_PATH" \
  lane_inference_image_size:=320 \
  lane_class_names:=center_line \
  lane_torch_threads:="$INTRA_THREADS" \
  lane_torch_interop_threads:="$INTEROP_THREADS" \
  lane_cpu_affinity:="$CPU_AFFINITY" \
  lane_yolo_device:=cpu \
  lane_yolo_fp16:=false \
  lane_torch_compile_mode:=none \
  lane_compile_warmup_iterations:=20 \
  lane_yolo_runtime_report_path:="$RUNTIME_JSON" \
  publish_pipeline_timing:=true \
  publish_performance_stats:=true \
  performance_log_period_s:=5.0 \
  publish_resource_stats:="$RESOURCE_ENABLED" \
  resource_stats_rate_hz:="$RESOURCE_HZ" \
  resource_stats_output_path:="$RESOURCE_CSV" \
  resource_stats_process_pattern:=lane_yolo_node \
  resource_stats_profile_threads:="$PROFILE_THREADS" \
  >"$RUN_DIR/launch.log" 2>&1 &
LAUNCH_PID=$!

for _ in $(seq 1 120); do
  if [ -f "$RUNTIME_JSON" ] && \
     ros2 service list 2>/dev/null | grep -Eq '^/start_integrated_drive$'; then
    break
  fi
  if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo "Launch exited before readiness; inspect $RUN_DIR/launch.log"
    exit 5
  fi
  sleep 1
done
if [ ! -f "$RUNTIME_JSON" ]; then
  echo "YOLO runtime report did not appear before timeout."
  exit 5
fi

python3 - "$RUNTIME_JSON" "$INTRA_THREADS" "$INTEROP_THREADS" <<'PY'
import json
import sys

manifest = json.load(open(sys.argv[1], encoding='utf-8'))
threads = manifest.get('cpu_thread_configuration', {})
expected = (int(sys.argv[2]), int(sys.argv[3]))
actual = (
    threads.get('actual_intra_op_threads'),
    threads.get('actual_inter_op_threads'),
)
if manifest.get('device') != 'cpu':
    raise SystemExit(f"Expected CPU backend, got {manifest.get('device')}")
if manifest.get('model_parameter_dtype') != 'float32':
    raise SystemExit('Expected FP32 model parameters')
if actual != expected:
    raise SystemExit(f'PyTorch thread mismatch: expected={expected} actual={actual}')
PY

ros2 service call /start_integrated_drive std_srvs/srv/Trigger '{}' \
  >"$RUN_DIR/start_service.log" 2>&1

ros2 bag record --storage sqlite3 --use-sim-time \
  --max-cache-size 104857600 \
  -o "$OUTPUT_BAG" \
  /clock /image_raw /perception/lane/image \
  /lane_yolo/detections /scene_yolo/detections \
  /center_curve /xycar_state_stamped /lane_control_state_stamped \
  /stanley/debug /lane_motor_cmd /lane_motor_cmd_stamped \
  /pipeline_timing /cpu_ab/motor /mission_status /mission_mode /rosout \
  >"$RUN_DIR/record.log" 2>&1 &
RECORDER_PID=$!
sleep 2

ros2 bag play "$BAG_PATH" --clock "$CLOCK_HZ" --rate 1.0 --delay 3 \
  --topics "${PLAY_TOPICS[@]}" --wait-for-all-acked 1000 \
  --disable-keyboard-controls \
  >"$RUN_DIR/play.log" 2>&1 &
PLAYER_PID=$!
if [ "$PROFILER_MODE" = on ]; then
  for _ in $(seq 1 30); do
    if ros2 service list 2>/dev/null | \
        grep -Eq '^/resource_profiler/refresh_processes$'; then
      break
    fi
    sleep 0.1
  done
  ros2 service call /resource_profiler/refresh_processes \
    std_srvs/srv/Trigger '{}' \
    >"$RUN_DIR/profiler_refresh.log" 2>&1
fi
wait "$PLAYER_PID"
PLAYER_PID=
sleep 2

ros2 service call /stop_integrated_drive std_srvs/srv/Trigger '{}' \
  >"$RUN_DIR/stop_service.log" 2>&1 || true
cleanup
trap - EXIT INT TERM

if [ "$PROFILER_MODE" = on ]; then
  python3 "$(dirname "$0")/analyze_yolo_resource_correlation.py" \
    "$OUTPUT_BAG" --resources "$RESOURCE_CSV" \
    --output-dir "$RUN_DIR/resource_analysis" \
    >"$RUN_DIR/resource_analysis.log" 2>&1
fi

echo "C1 CPU replay complete: $RUN_DIR"
