#include <bobnet_control/StaticController.h>

#include <bobnet_config/utils.h>

#include <vector>

namespace bobnet_control {

StaticController::StaticController(std::shared_ptr<JointPID> &pidControllerPtr,
                                   std::shared_ptr<StateSubscriber> &stateSubscriberPtr)
    : pidControllerPtr_(pidControllerPtr),
      stateSubscriberPtr_(stateSubscriberPtr),
      alpha_(-1.0),
      controllerType_("SIT") {
    kp_ = bobnet_config::fromRosConfigFile<scalar_t>("static_controller/kp");
    kd_ = bobnet_config::fromRosConfigFile<scalar_t>("static_controller/kd");

    // Load stand joint angles
    standJointAngles_ = bobnet_config::fromRosConfigFile<vector_t>("static_controller/stand_controller/joint_angles");

    // Load sit joint angles
    sitJointAngles_ = bobnet_config::fromRosConfigFile<vector_t>("static_controller/sit_controller/joint_angles");

    interpolationTime_ = bobnet_config::fromRosConfigFile<scalar_t>("static_controller/interpolation_time");

    ROS_INFO_STREAM("Initial controller" << controllerType_);
}

void StaticController::sendCommand(const scalar_t dt) {
    if (alpha_ != -1.0) {
        publishInterp(dt);
    } else if (controllerType_ == "STAND") {
        publishStand();
    } else if (controllerType_ == "SIT") {
        publishSit();
    } else {
        throw std::runtime_error("Unsupported controller type: " + controllerType_);
    }
}

void StaticController::visualize() {}

void StaticController::changeController(const std::string &controllerType) {
    controllerType_ = controllerType;
    alpha_ = 0.0;
    auto currentState = stateSubscriberPtr_->getState();
    interpFrom_ = currentState.jointPositions;
    if (controllerType_ == "STAND") {
        interpTo_ = standJointAngles_;
    } else if (controllerType_ == "SIT") {
        interpTo_ = sitJointAngles_;
    } else {
        throw std::runtime_error("Unsupported controller type: 2" + controllerType_);
    }
}

bool StaticController::isSupported(const std::string &controllerType) {
    if (controllerType == "STAND" || controllerType == "SIT") {
        return true;
    }
    return false;
}

void StaticController::publishStand() { pidControllerPtr_->sendCommand(standJointAngles_, kp_, kd_); }

void StaticController::publishSit() { pidControllerPtr_->sendCommand(sitJointAngles_, kp_, kd_); }

void StaticController::publishInterp(const scalar_t dt) {
    // update alpha
    alpha_ = std::min(alpha_ + dt / interpolationTime_, static_cast<scalar_t>(1.0));

    // compute new angles
    vector_t interpJointAngles = (1.0 - alpha_) * interpFrom_ + alpha_ * interpTo_;

    // send command
    pidControllerPtr_->sendCommand(interpJointAngles, kp_, kd_);

    // check if interpolation is done
    if (alpha_ == 1.0) {
        alpha_ = -1.0;
    }
}

}  // namespace bobnet_control