# FSM state of the stuck recovery supervisor.
#
# Values are stable -- do NOT renumber existing states.  They are plotted from bags
# and referenced by the RViz overlay and the operator UI.

builtin_interfaces/Time stamp

int32 NOMINAL=0
int32 SUSPECT=1
int32 PROBE=2
int32 RECOVERY=3
int32 BLOCKED=4
int32 COOLDOWN=5
int32 ABORT=6
# 7 was BACKOFF, a blind retreat along the path the robot came from.  Removed: it drove
# the robot against its own mission direction for as long as the obstacle kept coming.
# The number stays reserved so bags recorded before the removal still decode.
int32 BACKOFF_REMOVED=7
int32 REPLAN=8
# Every road back to lane driving runs through here: drive the reference path at the
# recovery crawl until the robot actually points along it.  Handing back on lateral
# distance alone let the robot leave at 57 degrees of heading error and drive off the path.
int32 TRANSIT=9

int32  state
string state_name

uint32  episode_id                  # increments on every NOMINAL -> SUSPECT
uint16  attempts                    # recovery attempts within the current episode
float32 elapsed_in_state_s
float32 elapsed_in_episode_s
# DEPRECATED, always 0.  There is no reverse budget any more: recovery refuses any plan
# that travels against the mission's motion mode, so "the obstacle closes in and the robot
# keeps giving ground" cannot happen and does not need bounding.  The fields remain so
# bags recorded before the removal still decode.
float32 reverse_distance_m
float32 total_reverse_distance_m
uint16  goal_index                  # which escape-goal candidate is being tried
uint16  goal_candidates             # how many were found
float32 lateral_excursion_m         # distance from the route centerline
string  abort_reason                # populated in BLOCKED / ABORT
