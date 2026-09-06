# Backward-compatible extension used by the Stanley controller.
# LaneControlState remains byte-compatible with previously recorded bags.
custom_interfaces/LaneControlState control
bool s_transition_active
float32 signed_curvature_hint
