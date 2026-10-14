# The part of the active route that geometric consumers (path building, ego lanelet lookup,
# drivable lanes) should plan on, so that a route passing over the same ground twice (an
# out-and-back on overlapping lanelets) never presents both passes at once.  Mission-level
# consumers (goal, remaining distance, route display) keep using the full route.
#
# Indices refer to LaneletRoute.segments of the route whose uuid is route_uuid.  A consumer
# whose route has a different uuid, or that gets no fresh message, uses the full route.

std_msgs/Header header
unique_identifier_msgs/UUID route_uuid

# false: ego could not be placed on the route (e.g. far off it); use the full route.
bool valid

# First and last segment of the window, both inclusive.
uint32 begin_segment
uint32 end_segment

# The segment ego is on, and its preferred primitive.
uint32 ego_segment
int64 ego_lanelet_id
