# Turn-in-place supervisor status.

uint8 MONITOR = 0                 # heading follows the route, nothing to do
uint8 REQUEST_STOP = 1            # heading is wrong; holding the robot until it stands still
uint8 CHECK_CLEARANCE = 2         # standing still; waiting for a side free to rotate through
uint8 TURNING = 3                 # executor is turning the robot
uint8 COOLDOWN = 4                # turn done; holding a moment before handing back
uint8 ERROR = 5                   # a phase timed out; the robot is held still
uint8 REVERSING_TO_TURN_SPOT = 6  # the road is too narrow to turn here: backing up along the
                                  # route (towards the goal) to the first spot wide enough

builtin_interfaces/Time stamp
uint8 state
float64 yaw_error           # [rad] route heading at ego - body heading, normalized
float64 target_yaw          # [rad] route heading at ego (map frame)
int8 direction              # TurnInPlaceCommand.DIRECTION_*, 0 when not turning
bool ccw_clear              # rotating counter-clockwise is safe (road area and obstacles)
bool cw_clear               # rotating clockwise is safe (road area and obstacles)
bool ccw_inside_road        # the counter-clockwise sweep stays inside the road lanelets
bool cw_inside_road         # the clockwise sweep stays inside the road lanelets
bool turn_spot_found        # a spot wide enough to turn lies ahead along the route
float64 turn_spot_distance  # [m] along the route from ego to that spot (when found)
bool holding_vehicle        # a zero velocity limit is being published
string reason
