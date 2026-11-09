# Turn-in-place supervisor -> behavior_path_planner reverse_lane_follow (activation
# "external_request"): back up along the route (towards the goal), nose facing away from it.

builtin_interfaces/Time stamp
bool active                    # false: do not reverse
bool has_stop_pose             # true: stop at stop_pose; false: reverse all the way to the goal
geometry_msgs/Pose stop_pose   # map frame; only its position is used
