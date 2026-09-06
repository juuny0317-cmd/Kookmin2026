#!/usr/bin/env bash
set -Eeuo pipefail

readonly WORKSPACE='/home/xytron/xycar_ws'
readonly PINNED_CONTAINER_IMAGE='sha256:03acec7d3f671131801a951600d164566436e4a2b009fc109d755f2bd5a41627'
readonly CONTAINER_IMAGE="${XYCAR_IMAGE:-$PINNED_CONTAINER_IMAGE}"
readonly EXECUTION_BACKEND="${XYCAR_EXECUTION_BACKEND:-container}"
readonly START_MOTOR_DRIVER="${XYCAR_START_MOTOR_DRIVER:-true}"

case "$EXECUTION_BACKEND" in
  container|host) ;;
  *)
    echo 'XYCAR_EXECUTION_BACKEND must be container or host.' >&2
    exit 2
    ;;
esac

case "$START_MOTOR_DRIVER" in
  true|false) ;;
  *)
    echo 'XYCAR_START_MOTOR_DRIVER must be true or false.' >&2
    exit 2
    ;;
esac

set +u
source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

if [[ "$EXECUTION_BACKEND" == container ]]; then
  docker image inspect "$CONTAINER_IMAGE" >/dev/null 2>&1 || {
    echo "Docker image is missing or Docker is unavailable: $CONTAINER_IMAGE" >&2
    echo 'Run: bash ./scripts/build_jetson_mission_image.sh' >&2
    exit 1
  }
fi

pkill -TERM -f '[r]viz_visualizer_node' 2>/dev/null || true
pkill -TERM -x rviz2 2>/dev/null || true

if [[ "$EXECUTION_BACKEND" == container ]]; then
  docker ps -q \
    --filter 'label=com.xycar.role=integrated_drive' \
    | xargs -r docker stop -t 10
fi

echo "Image: $CONTAINER_IMAGE"
echo "Backend: $EXECUTION_BACKEND"
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "Motor driver: $START_MOTOR_DRIVER"

exec ros2 launch mission_cone_drive integrated_drive.launch.py \
  execution_backend:="$EXECUTION_BACKEND" \
  container_image:="$CONTAINER_IMAGE" \
  start_camera:=true \
  start_lidar:=true \
  start_yolo:=true \
  start_lane_stack:=true \
  enable_lane_pipeline:=true \
  enable_scene_pipeline:=true \
  enable_drive_control:=true \
  start_motor_driver:="$START_MOTOR_DRIVER" \
  start_ultrasonic:=false \
  start_active:=false \
  enable_spacebar_pause:=true \
  start_rviz:=false \
  show_scene_yolo_view:=false \
  scene_model_filename:=all_second_yolo_v2_yolov10n_640_best_e46.pt \
  scene_class_names:=cone,dynamic,green,left,red,static \
  scene_class_name_aliases:=dynamic=obstacle_vehicle \
  motor_port:=auto \
  vesc_handshake_timeout_s:=2.0 \
  vesc_reconnect_interval_s:=1.0 \
  vesc_telemetry_timeout_s:=1.0 \
  enable_shortcut_left_turn:=true \
  shortcut_auto_trigger_after_s:=-1.0 \
  shortcut_auto_trigger_state:=entry \
  shortcut_prepare_speed_limit:=12.0 \
  shortcut_turn_speed_limit:=12.0 \
  shortcut_cruise_speed_limit:=22.0 \
  shortcut_steering_limit:=39.0 \
  shortcut_left_signal_window:=5 \
  shortcut_left_signal_min_matches:=3 \
  shortcut_fallback_commit_s:=5.0 \
  shortcut_entry_max_heading_deg:=60.0 \
  shortcut_min_cruise_duration_s:=2.0 \
  shortcut_crossline_window:=3 \
  shortcut_crossline_min_matches:=2 \
  shortcut_exit_min_turn_duration_s:=1.0 \
  shortcut_exit_alignment_window:=5 \
  shortcut_exit_alignment_min_matches:=2 \
  shortcut_exit_handoff_duration_s:=0.0 \
  shortcut_exit_max_heading_deg:=60.0 \
  road_curve_max_time_delta_s:=0.25 \
  obstacle_lidar_min_range_m:=0.18 \
  obstacle_lidar_max_range_m:=5.0 \
  obstacle_lidar_cluster_min_points:=2 \
  obstacle_lidar_cluster_base_gap_m:=0.08 \
  obstacle_lidar_cluster_range_gap_scale:=1.5 \
  obstacle_lidar_cluster_max_width_m:=2.0 \
  obstacle_lidar_distance_percentile:=50.0 \
  obstacle_lidar_association_padding_px:=35.0 \
  obstacle_lidar_association_max_above_px:=10.0 \
  obstacle_lidar_camera_max_delta_s:=0.15 \
  obstacle_lidar_tracking_enabled:=true \
  obstacle_lidar_track_max_age_s:=0.60 \
  obstacle_lidar_track_max_misses:=3 \
  obstacle_lidar_track_max_distance_jump_m:=0.45 \
  obstacle_lidar_track_max_range_rate_mps:=3.0 \
  obstacle_lidar_track_max_total_distance_jump_m:=1.50 \
  obstacle_lidar_track_max_offset_jump_px:=55.0 \
  obstacle_lidar_track_max_offset_rate_px_s:=120.0 \
  obstacle_lidar_track_bbox_iou_min:=0.10 \
  obstacle_lidar_track_bbox_center_gate_px:=150.0 \
  obstacle_lidar_near_cluster_distance_m:=0.0 \
  obstacle_lidar_near_cluster_min_bbox_height_ratio:=0.16 \
  obstacle_lidar_near_cluster_min_bbox_bottom_ratio:=0.56 \
  obstacle_camera_fallback_speed_enabled:=true \
  obstacle_camera_fallback_min_bottom_ratio:=0.48 \
  obstacle_camera_event_enabled:=true \
  obstacle_camera_event_min_bottom_ratio:=0.51 \
  obstacle_camera_event_lane_margin_px:=12.0 \
  obstacle_allow_avoid_on_curve:=true \
  dynamic_obstacle_speed:=26.0 \
  dynamic_speed_trigger_distance_m:=4.0 \
  dynamic_overtake_trigger_distance_m:=4.0 \
  dynamic_lane_determination_distance_m:=4.0 \
  dynamic_event_duration_s:=2.0 \
  dynamic_speed_confirm_min_matches:=1 \
  dynamic_speed_confirm_window_frames:=7 \
  dynamic_avoid_confirm_min_matches:=1 \
  dynamic_avoid_confirm_window_frames:=7 \
  dynamic_rearm_clear_frames:=3 \
  static_obstacle_speed:=18.0 \
  static_speed_trigger_distance_m:=3.0 \
  static_overtake_trigger_distance_m:=1.5 \
  static_lane_determination_distance_m:=1.5 \
  static_event_duration_s:=1.2 \
  static_speed_confirm_min_matches:=1 \
  static_speed_confirm_window_frames:=3 \
  static_avoid_confirm_min_matches:=1 \
  static_avoid_confirm_window_frames:=3 \
  static_rearm_clear_frames:=5 \
  lane_max_rate_hz:=15 \
  lane_yolo_max_rate_hz:=0.0 \
  lane_inference_image_size:=320 \
  scene_max_rate_hz:=10 \
  scene_yolo_max_rate_hz:=0.0 \
  scene_inference_image_size:=640 \
  camera_qos_reliability:=best_effort \
  lane_image_qos_reliability:=best_effort \
  scene_image_qos_reliability:=reliable \
  scene_process_nice:=0 \
  publish_performance_stats:=true \
  performance_log_period_s:=3.0 \
  frame_router_event_driven:=true \
  stanley_event_driven:=true \
	  mission_event_driven_lane:=true \
	  lane_straight_speed:=30.0 \
	  lane_curve_strong_speed:=13.0 \
	  lane_curve_medium_speed:=13.0 \
	  lane_curve_mild_speed:=13.0 \
	  lane_straight_recovery_speed:=20.0 \
	  lane_curve_strong_threshold:=0.60 \
	  lane_curve_medium_threshold:=0.20 \
	  lane_straight_enter_cte_px:=50.0 \
	  lane_straight_exit_cte_px:=65.0 \
	  lane_recovery_max_cte_px:=120.0 \
	  lane_recovery_max_heading_deg:=25.0 \
	  lane_curve_accel_rate:=300.0 \
	  lane_straight_accel_rate:=300.0 \
	  lane_curve_decel_rate:=300.0 \
	  lane_straight_decel_rate:=300.0 \
  lane_curve_heading_recovery_enabled:=true \
  lane_curve_heading_disagreement_deg:=25.0 \
  lane_curve_heading_recovery_weight:=0.65 \
  lane_curve_heading_confirm_frames:=2 \
  lane_adaptive_heading_enabled:=true \
  lane_heading_preview_min_ratio:=0.08 \
  lane_heading_preview_max_ratio:=0.16 \
  lane_heading_curvature_scale:=0.50 \
  lane_heading_curvature_deadzone:=0.0 \
  lane_heading_filter_alpha_straight:=0.65 \
  lane_heading_filter_alpha_curve:=0.70 \
	  lane_heading_filter_alpha_fallback:=0.35 \
	  lane_heading_curve_hold_frames:=3 \
	  lane_heading_max_step_deg:=12.0 \
	  lane_straight_center_agreement_threshold_px:=10.0 \
	  lane_curve_center_agreement_threshold_px:=5.0 \
	  lane_curve_center_correction_weight:=0.95 \
	  lane_lateral_innovation_straight_px:=35.0 \
  lane_lateral_innovation_curve_px:=120.0 \
  lane_lateral_max_rate_straight_px_s:=320.0 \
  lane_lateral_lane_half_width_px:=250.0 \
  lane_lateral_outer_pair_tolerance_px:=80.0 \
  lane_lateral_outer_pair_hold_frames:=5 \
  stanley_control_rate_hz:=20.0 \
  mission_decision_rate_hz:=20.0 \
  stanley_state_timeout_s:=0.30 \
  stanley_hold_last_angle_when_stale:=true \
  stanley_curvature_steering_rate_enabled:=false \
  stanley_steering_rate_straight_per_s:=200.0 \
  stanley_steering_rate_curve_per_s:=300.0 \
  stanley_steering_rate_reversal_per_s:=360.0 \
  stanley_steering_rate_risk_low:=0.15 \
  stanley_steering_rate_risk_high:=0.60 \
  stanley_steering_rate_hold_frames:=4 \
  stanley_gain_schedule_start_speed:=4.0 \
  stanley_gain_schedule_end_speed:=12.0 \
  stanley_k_straight_low_speed:=1.0 \
  stanley_k_straight_high_speed:=0.65 \
  stanley_k_curve_low_speed:=1.20 \
  stanley_k_curve_high_speed:=0.90 \
  stanley_k_obstacle:=1.80 \
  stanley_adaptive_heading_weight_enabled:=true \
  stanley_heading_weight_low_speed:=0.30 \
  stanley_heading_weight_straight_high_speed:=0.18 \
  stanley_heading_weight_hough_high_speed:=0.14 \
  stanley_heading_weight_curve_high_speed:=0.30 \
  stanley_heading_weight_rise_alpha:=0.65 \
  stanley_heading_weight_fall_alpha:=0.35 \
  stanley_accel_rate_per_s:=300.0 \
  stanley_decel_rate_per_s:=300.0 \
  traffic_direct_red_min_confidence:=0.30 \
  traffic_direct_left_min_confidence:=0.60 \
  traffic_direct_green_min_confidence:=0.50 \
  traffic_left_is_green:=true \
  traffic_red_latch_until_green:=true \
  traffic_red_latch_confirm_frames:=2 \
  cone_perception_mode:=lidar_dbscan \
  cone_controller:=pure_pursuit \
  cone_min_speed:=7.0 \
  cone_max_speed:=7.0 \
  cone_entry_speed:=7.0 \
  cone_speed_accel_rate_per_s:=300.0 \
  cone_speed_decel_rate_per_s:=300.0 \
  cone_conf_threshold:=0.15 \
  cone_entry_use_fusion_gate:=true \
  cone_entry_allow_lidar_fallback:=true \
  cone_entry_min_fused_clusters:=2 \
  cone_entry_require_both_sides:=true \
  cone_entry_side_min_abs_y_m:=0.10 \
  cone_entry_fusion_timeout_s:=0.30 \
  cone_entry_fusion_max_range_m:=1.50 \
  cone_entry_fusion_rotation_deg:=-125.0 \
  cone_entry_fusion_projection_model:=fisheye \
  bbox_padding_x_px:=20.0 \
  bbox_padding_y_px:=30.0 \
  cone_prearm_enabled:=true \
  cone_bbox_prearm_enabled:=true \
  cone_prearm_use_fusion:=false \
  cone_prearm_max_range_m:=4.00 \
  cone_prearm_min_bbox_area_px:=270.0 \
  cone_prearm_min_bottom_y_px:=170.0 \
  cone_bbox_prearm_min_detections:=1 \
  cone_bbox_prearm_clear_frames:=3 \
  cone_prearm_min_clusters:=1 \
  cone_prearm_window:=3 \
  cone_prearm_min_matches:=2 \
  cone_prearm_speed_cap:=12.0 \
  cone_prearm_dbscan_eps_m:=0.20 \
  cone_prearm_dbscan_min_samples:=3 \
  cone_suspect_min_fused_clusters:=1 \
  cone_suspect_hold_s:=0.70 \
  cone_suspect_speed_cap:=0.0 \
  lidar_rotation_deg:=0.0 \
  lidar_dbscan_min_range_m:=0.18 \
  lidar_dbscan_max_range_m:=1.50 \
  lidar_dbscan_eps_m:=0.04 \
  lidar_dbscan_min_samples:=3 \
  lidar_dbscan_max_cone_diameter_m:=0.30 \
  lidar_dbscan_entry_min_clusters:=3 \
  lidar_dbscan_allow_cone_reentry:=true \
  cone_entry_min_detections:=2 \
  cone_entry_min_path_points:=5 \
  cone_enter_confirm_cycles:=3 \
  cone_entry_min_bottom_y_px:=220.0 \
  cone_min_mode_duration_s:=3.0 \
  cone_exit_hold_s:=0.1
