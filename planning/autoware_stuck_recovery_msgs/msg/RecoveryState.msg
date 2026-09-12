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
int32 BACKOFF=7
int32 REPLAN=8

int32  state
string state_name

uint32  episode_id                  # increments on every NOMINAL -> SUSPECT
uint16  attempts                    # recovery attempts within the current episode
float32 elapsed_in_state_s
float32 elapsed_in_episode_s
float32 reverse_distance_m          # reverse travel this episode
# Reverse travel accumulated across episodes, reset only by real forward progress toward
# the goal.  This is what bounds the "obstacle keeps advancing, robot keeps retreating"
# case, which a per-episode budget cannot see: each new episode would hand out a fresh
# allowance and the total would be unbounded.
float32 total_reverse_distance_m
uint16  goal_index                  # which escape-goal candidate is being tried
uint16  goal_candidates             # how many were found
float32 lateral_excursion_m         # distance from the route centerline
string  abort_reason                # populated in BLOCKED / ABORT
