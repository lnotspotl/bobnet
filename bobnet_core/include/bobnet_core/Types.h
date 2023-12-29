#pragma once

#include <vector>
#include <Eigen/Dense>

namespace bobnet_core {

using scalar_t = double;
using vs = std::vector<scalar_t>;
using vvs = std::vector<vs>;

struct VelocityCommand {
    scalar_t velocity_x = 0.0;
    scalar_t velocity_y = 0.0;
    scalar_t yaw_rate = 0.0;
};

} // namespace bobnet