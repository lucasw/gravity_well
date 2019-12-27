# gravity_well
Gravity golf/artillery with increasing complex gravity fields

Multiple players can direct their cannons/launch facilities to launch a projectile or a rocket with a given yaw and pitch and velocity or constant acceleration (and later acceleration profiles, course corrections) and attempt to hit a target (possibly the location of the other player).

No integrated visualization engine- use ROS rviz and imgui_ros viz3d.

TF reflect the current position of the rockets and all current gravity wells.

rviz Markers will contain the past trajectories (maybe create a generic rviz plugin for trajectory tracking would be good, put it in a separate repo).

Possibly use bullet (via simple_sim_ros) for collision detection, or use simple sphere intersection initially.

Use ros for giving commands: a basic message giving the angles and initial impulse would come first.

Also could make use of rviz interactive markers for launch commands.
