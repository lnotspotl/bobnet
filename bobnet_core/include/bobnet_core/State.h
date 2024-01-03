#pragma once

#include <Eigen/Dense>
#include <bobnet_msgs/RobotState.h>

#include <bobnet_core/Types.h>

namespace bobnet_core {

struct State {
    using Vector3 = Eigen::Matrix<scalar_t, 3, 1>;
    using Vector4 = Eigen::Matrix<scalar_t, 4, 1>;
    using Vector12 = Eigen::Matrix<scalar_t, 12, 1>;

    Vector3 basePositionWorld;
    Vector4 baseOrientationWorld;  // quaternion, xyzw
    Vector3 baseLinearVelocityBase;
    Vector3 baseAngularVelocityBase;
    Vector3 normalizedGravityBase;
    Vector12 jointPositions;
    Vector12 jointVelocities;
    Vector3 lfFootPositionWorld;
    Vector3 lhFootPositionWorld;
    Vector3 rfFootPositionWorld;
    Vector3 rhFootPositionWorld;

    static State fromMessage(const bobnet_msgs::RobotState::ConstPtr stateMsg);
};

}  // namespace bobnet_core