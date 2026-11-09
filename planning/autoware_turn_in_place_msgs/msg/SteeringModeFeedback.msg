# Steering-mode feedback from the chassis (or its simulator mock).
# Mode numbers follow middleware_controller_manager: 0=FWS 1=RWS 2=4WCS 3=CRAB 4=HOLONOMIC.

builtin_interfaces/Time stamp
uint8 mode          # mode currently in force
uint8 target_mode   # mode last requested
bool switching      # wheels are still re-orienting to target_mode; the chassis does not move
