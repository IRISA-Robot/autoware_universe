# pedestrian_light_arbiter status, /road_crossing/light_override/status (2026-11-17).
std_msgs/Header header
uint8 mode                      # active mode, PedestrianLightOverride.AUTO / RED / GREEN
uint32 request_id               # request that set the active mode (0 in AUTO by default)
string requester                # who set the active mode
float32 green_lease_remaining_sec  # GREEN only, else -1
uint32 last_used_green_id         # last GREEN request used up / lapsed (a page resets its button)
uint8 detected_state            # last PedestrianLightState.state from the detector
bool detected_fresh             # detector message newer than detector_timeout_sec
uint8 output_state              # PedestrianLightState.state the FSM gets now
string reason                   # why the mode last changed
