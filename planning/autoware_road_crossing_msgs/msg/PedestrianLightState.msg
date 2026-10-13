std_msgs/Header header
uint8 NONE=0
uint8 RED=1
uint8 GREEN=2
uint8 COUNTDOWN_GREEN=3
uint8 COUNTDOWN_BLANK=4
uint8 state
float32 confidence
bool blocked
# tf_pedestrian way id of the light this state belongs to (0 = no light / unknown).
# The FSM accepts a non-NONE state only from a light paired with the active crossing.
int64 light_id
