# Robot state message

float64[3] base_position_world
float64[4] base_orientation_world  # x y z w

float64[3] base_lin_vel_b
float64[3] base_ang_vel_b
float64[3] normalized_gravity_b
float64[12] joint_pos
float64[12] joint_vel

float64[3] lf_position_world
float64[3] lh_position_world
float64[3] rf_position_world
float64[3] rh_position_world