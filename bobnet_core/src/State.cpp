#include <bobnet_core/State.h>

namespace bobnet_core {

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
State State::fromMessage(const bobnet_msgs::RobotState::ConstPtr stateMsg) {
    State state;
    state.basePositionWorld = Eigen::Map<const State::Vector3>(stateMsg->base_position_world.data());
    state.baseOrientationWorld = Eigen::Map<const State::Vector4>(stateMsg->base_orientation_world.data());
    state.baseLinearVelocityBase = Eigen::Map<const State::Vector3>(stateMsg->base_lin_vel_b.data());
    state.baseAngularVelocityBase = Eigen::Map<const State::Vector3>(stateMsg->base_ang_vel_b.data());
    state.normalizedGravityBase = Eigen::Map<const State::Vector3>(stateMsg->normalized_gravity_b.data());
    state.jointPositions = Eigen::Map<const State::Vector12>(stateMsg->joint_pos.data());
    state.jointVelocities = Eigen::Map<const State::Vector12>(stateMsg->joint_vel.data());
    state.lfFootPositionWorld = Eigen::Map<const State::Vector3>(stateMsg->lf_position_world.data());
    state.lhFootPositionWorld = Eigen::Map<const State::Vector3>(stateMsg->lh_position_world.data());
    state.rfFootPositionWorld = Eigen::Map<const State::Vector3>(stateMsg->rf_position_world.data());
    state.rhFootPositionWorld = Eigen::Map<const State::Vector3>(stateMsg->rh_position_world.data());
    return state;
}


}  // namespace bobnet_core