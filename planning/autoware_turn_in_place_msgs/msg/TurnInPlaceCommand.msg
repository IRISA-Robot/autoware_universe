# Request from the turn-in-place supervisor to the executor.
# active=false cancels any turn in progress (the executor stops rotating and restores its mode).

int8 DIRECTION_CCW = 1   # yaw increasing (turn left)
int8 DIRECTION_CW = -1   # yaw decreasing (turn right)

builtin_interfaces/Time stamp
bool active
float64 target_yaw       # [rad] map frame heading the body must reach
int8 direction           # DIRECTION_CCW or DIRECTION_CW: the side to rotate towards
