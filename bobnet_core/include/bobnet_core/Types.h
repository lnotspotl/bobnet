#pragma once

#include <vector>
#include <Eigen/Dense>

namespace bobnet_core {

using scalar_t = double;  // double = RobotState float64, float = RobotState float32 !!!
using vs = std::vector<scalar_t>;
using vvs = std::vector<vs>;

using vector3_t = Eigen::Matrix<scalar_t, 3, 1>;
using vector_t = Eigen::Matrix<scalar_t, Eigen::Dynamic, 1>;

using matrix3_t = Eigen::Matrix<scalar_t, 3, 3>;
using matrix_t = Eigen::Matrix<scalar_t, Eigen::Dynamic, Eigen::Dynamic>;

using quaternion_t = Eigen::Quaternion<scalar_t>;
using angleaxis_t = Eigen::AngleAxis<scalar_t>;

struct VelocityCommand {
    scalar_t velocity_x = 0.0;
    scalar_t velocity_y = 0.0;
    scalar_t yaw_rate = 0.0;
};

} // namespace bobnet