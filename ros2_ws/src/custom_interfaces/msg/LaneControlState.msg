# Lane tracking state and the perception evidence used to create it.
std_msgs/Header header
custom_interfaces/XycarState state
string curve_state
string heading_source
bool curve_heading_valid
bool s_reversal_active
float32 curvature
float32 curvature_severity
float32 heading_preview_ratio
float32 curve_fit_error_ratio
float32 path_confidence
