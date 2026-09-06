# Backward-compatible speed/recovery diagnostics. V1/V2/V3 stay unchanged.
custom_interfaces/StanleyDebugV3 planner
string speed_reason
float32 geometry_target_speed
float32 speed_before_rate_limit
float32 speed_after_rate_limit
# Negative means that no phase-specific hard cap is active.
float32 active_speed_cap
bool alignment_recovery_active
bool alignment_ready
string alignment_recovery_subphase
bool heading_cte_conflict
bool recovery_cte_conflict_cap_applied
float32 recovery_cte_ratio
bool s_candidate_active
bool s_confirmed
bool spatial_s_reversal
bool temporal_s_reversal
string s_evidence
