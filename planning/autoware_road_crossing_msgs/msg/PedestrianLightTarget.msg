# Pedestrian light(s) paired with the crossing the road_crossing FSM is handling.
# Published by the FSM every tick; detector, rotary aimer and the simulator follow it
# instead of picking the nearest light on their own.
#
# The pairing comes from the road_crossing lanelet, keyed by direction of travel:
#   forward_pedestrian_tf = "<way id>[,<way id>...]"
#   reverse_pedestrian_tf = "<way id>[,<way id>...]"
std_msgs/Header header
# Active road_crossing lanelet, -1 when no episode is active.
int64 crossing_id
uint8 FORWARD=0
uint8 REVERSE=1
uint8 direction
# Paired tf_pedestrian way ids; empty = no episode, or a crossing without lights.
int64[] light_ids
