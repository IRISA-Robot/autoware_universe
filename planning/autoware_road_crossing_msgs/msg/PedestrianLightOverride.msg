# Operator command to pedestrian_light_arbiter (2026-11-17).  Any source -- mock RC web, a
# remote node -- publishes this on /road_crossing/light_override.
#   AUTO  = pass the detector through (default).
#   RED   = force red: the robot waits at the crossing.  Stays until AUTO (no lease).
#   GREEN = force green: the robot may cross.  Must be re-sent within the arbiter's
#           green_lease_sec or it lapses to AUTO; it is also used up when a crossing ends.
#           A used-up / lapsed request_id is not taken again: press GREEN again (new id).
std_msgs/Header header
uint8 AUTO=0
uint8 RED=1
uint8 GREEN=2
uint8 mode
# Changes on every new operator press (same id = the same press, re-sent as a heartbeat).
uint32 request_id
# Who sent it (e.g. "mock_rc_web 192.168.1.5"), for the log and the status topic.
string requester
