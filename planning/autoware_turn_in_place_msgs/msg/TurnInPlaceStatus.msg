# Executor -> supervisor.

uint8 IDLE = 0
uint8 SWITCHING_MODE = 1   # changing the steering mode to the spin mode, wheels re-orienting
uint8 ROTATING = 2
uint8 RESTORING_MODE = 3   # changing back to the driving steering mode
uint8 DONE = 4             # target reached and driving mode restored
uint8 FAILED = 5

builtin_interfaces/Time stamp
uint8 state
float64 yaw_error          # [rad] target_yaw - current yaw, normalized
string reason
