#include <bobnet_core/State.h>

namespace bobnet_core {

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
State State::fromMessage(const bobnet_msgs::RobotState::ConstPtr stateMsg) {
    State state;
    state.basePositionWorld = Eigen::Map<const Eigen::Vector3d>(stateMsg->base_position_world.data());
    state.baseOrientationWorld = Eigen::Map<const Eigen::Vector4d>(stateMsg->base_orientation_world.data());
    state.baseLinearVelocityBase = Eigen::Map<const Eigen::Vector3d>(stateMsg->base_lin_vel_b.data());
    state.baseAngularVelocityBase = Eigen::Map<const Eigen::Vector3d>(stateMsg->base_ang_vel_b.data());
    state.normalizedGravityBase = Eigen::Map<const Eigen::Vector3d>(stateMsg->normalized_gravity_b.data());
    state.jointPositions = Eigen::Map<const State::Vector12d>(stateMsg->joint_pos.data());
    state.jointVelocities = Eigen::Map<const State::Vector12d>(stateMsg->joint_vel.data());
    state.lfFootPositionWorld = Eigen::Map<const Eigen::Vector3d>(stateMsg->lf_position_world.data());
    state.lhFootPositionWorld = Eigen::Map<const Eigen::Vector3d>(stateMsg->lh_position_world.data());
    state.rfFootPositionWorld = Eigen::Map<const Eigen::Vector3d>(stateMsg->rf_position_world.data());
    state.rhFootPositionWorld = Eigen::Map<const Eigen::Vector3d>(stateMsg->rh_position_world.data());
    return state;
}

}  // namespace bobnet_core