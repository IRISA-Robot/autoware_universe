# Observability output of the stuck watchdog.  Published every tick so that a bag
# replay can reconstruct exactly why the supervisor did (or did not) act.

builtin_interfaces/Time stamp

# --- motion ---------------------------------------------------------------
bool    is_stopped                  # |v_measured| < th_stopped_velocity_mps
float32 stopped_duration_s          # how long that has been continuously true
float32 commanded_velocity_mps      # /control/command/control_cmd
float32 measured_velocity_mps       # /vehicle/status/velocity_status
float32 progress_since_entry_m      # cumulative distance from the SUSPECT entry pose

# --- why are we stopped ---------------------------------------------------
uint8 SOURCE_NONE=0
uint8 SOURCE_OBSTACLE_STOP=1        # pathological: obstacle_stop & friends
uint8 SOURCE_MAP_STOP=2             # legit: stop_line / crosswalk / walkway / detection_area
uint8 SOURCE_MECHANICAL=3           # commanded to move but not moving -> never recover
uint8 SOURCE_PLANNER_EMPTY=4        # no trajectory at all
uint8 SOURCE_UNKNOWN=5
# A STOP factor whose control point lies BEHIND the robot.  This does NOT mean the
# obstacle is behind: it means the robot is inside the obstacle's stop margin, so the
# required stop line falls back where it has already been.  insert_stop() then zeroes the
# trajectory from that index to the end, freezing everything ahead -- lane driving has no
# way out, which makes this the case recovery matters most for.
uint8 SOURCE_STOP_BEHIND=6
uint8  stop_source
string stop_source_module           # planning factor topic that won the classification

bool   legit_stop                   # true -> suppress the watchdog entirely
string legit_reason

# --- geometry -------------------------------------------------------------
# Left and right are measured and aggregated independently along the corridor, and
# are never summed: a usable gap means "the robot fits on THIS side for the whole
# stretch".  free_width_m is whichever single side is better, not a mix.
uint8 SIDE_NONE=0
uint8 SIDE_LEFT=1
uint8 SIDE_RIGHT=2
# True on a reversed route: the path yaw is the direction of travel while the robot
# faces the other way, so left/right below are mirrored back into the ROBOT's frame.
bool    path_reversed
# Left and right are the ROBOT's left and right.
float32 free_left_m
float32 free_right_m
float32 free_width_m                # = max(free_left_m, free_right_m)
uint8   chosen_side
float32 chosen_offset_m             # signed lateral offset (+left) to that gap's centre
float32 required_width_m            # vehicle_width + 2 * lateral_margin_m
bool    corridor_clear              # EXIT check: reference path ahead is unobstructed
# The robot's own footprint is inside an (inflated) obstacle, so freespace rejects the
# START pose outright and no goal can ever help.  This is what triggers a backoff.
bool    start_blocked
float32 remaining_distance_m        # to the mission goal
