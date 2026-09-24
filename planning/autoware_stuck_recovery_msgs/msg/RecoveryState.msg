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
# 6 was ABORT, a latched failure.  Giving up now goes straight to COOLDOWN and the next
# episode starts afresh; this value is never published.  Kept so older bags still decode.
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
# The ground budget.  Freespace plans may back up as part of getting past (never a blind
# retreat -- the plan must end going the mission's way), and this is what bounds it.
# reverse_distance_m: how far back along the reference path the robot is from the furthest
#   point it reached this episode; checked against max_ground_given_m.
# total_reverse_distance_m: everything driven against the mission this session.
# Always 0 in bags recorded between the BACKOFF removal and this change.
float32 reverse_distance_m
float32 total_reverse_distance_m
uint16  goal_index                  # which escape-goal candidate is being tried
uint16  goal_candidates             # how many were found
float32 lateral_excursion_m         # distance from the route centerline
string  abort_reason                # populated in BLOCKED, and in COOLDOWN after giving up
