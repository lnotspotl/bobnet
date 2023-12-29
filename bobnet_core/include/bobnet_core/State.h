#pragma once

#include <Eigen/Dense>
#include <bobnet_msgs/RobotState.h>

namespace bobnet_core {

struct State {
    using Vector12d = Eigen::Matrix<double, 12, 1>;

    Eigen::Vector3d basePositionWorld;
    Eigen::Vector4d baseOrientationWorld;  // quaternion, xyzw
    Eigen::Vector3d baseLinearVelocityBase;
    Eigen::Vector3d baseAngularVelocityBase;
    Eigen::Vector3d normalizedGravityBase;
    Vector12d jointPositions;
    Vector12d jointVelocities;
    Eigen::Vector3d lfFootPositionWorld;
    Eigen::Vector3d lhFootPositionWorld;
    Eigen::Vector3d rfFootPositionWorld;
    Eigen::Vector3d rhFootPositionWorld;

    static State fromMessage(const bobnet_msgs::RobotState::ConstPtr stateMsg);
};

}  // namespace bobnet_core