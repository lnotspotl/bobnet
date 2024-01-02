#include <bobnet_controllers/Controllers.h>

#include <chrono>

#define WARN(x) ROS_WARN_STREAM("[RL controller] " << x);

namespace bobnet_controllers {


inline int mod(int a, int b) {
    return (a % b + b) % b;
}

/*******************************************************************************/
/*******************************************************************************/
/*******************************************************************************/
StandController::StandController(std::vector<std::string> jointNames, vector_t jointAngles, scalar_t kp,
                                 scalar_t kd)
    : jointNames_(jointNames), jointAngles_(jointAngles), kp_(kp), kd_(kd) {}

/*******************************************************************************/
/*******************************************************************************/
/*******************************************************************************/
bobnet_msgs::JointCommandArray StandController::getCommandMessage(const State &state, scalar_t dt) {
    bobnet_msgs::JointCommandArray commandArray;
    commandArray.joint_commands.resize(jointNames_.size());

    for (size_t i = 0; i < jointNames_.size(); ++i) {
        commandArray.joint_commands[i].joint_name = jointNames_[i];
        commandArray.joint_commands[i].position_desired = jointAngles_[i];
        commandArray.joint_commands[i].velocity_desired = 0.0;
        commandArray.joint_commands[i].kp = kp_;
        commandArray.joint_commands[i].kd = kd_;
        commandArray.joint_commands[i].torque_ff = 0.0;
    }

    return commandArray;
}

/*******************************************************************************/
/*******************************************************************************/
/*******************************************************************************/
std::string controllerType2String(ControllerType type) {
    switch (type) {
        case ControllerType::STAND:
            return "STAND";
        case ControllerType::RL:
            return "RL";
        default:
            throw std::runtime_error("Unknown controller type");
    }
}

/*******************************************************************************/
/*******************************************************************************/
/*******************************************************************************/
ControllerType string2ControllerType(const std::string &type) {
    if (type == "STAND") {
        return ControllerType::STAND;
    } else if (type == "RL") {
        return ControllerType::RL;
    } else {
        throw std::runtime_error("Unknown controller type");
    }
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
bobnet_msgs::JointCommandArray RlController::getCommandMessage(const State &state, scalar_t dt) {
    // Do not keep track of gradients
    torch::NoGradGuard no_grad;

    auto ts1 = std::chrono::high_resolution_clock::now();
    auto nnInput = getNNInput(state, dt);
    auto t2 = std::chrono::high_resolution_clock::now();

    ROS_INFO_STREAM_THROTTLE(
        1.0, "NN input computation took: "
                 << std::chrono::duration_cast<std::chrono::microseconds>(t2 - ts1).count() / 1000.0 << " ms");

    // perform forward pass
    auto ts3 = std::chrono::high_resolution_clock::now();
    at::Tensor out = model_.forward({nnInput.view({1, getNNInputSize()})}).toTensor().squeeze();
    auto t4 = std::chrono::high_resolution_clock::now();

    ROS_INFO_STREAM_THROTTLE(
        1.0,
        "NN forward pass took: " << std::chrono::duration_cast<std::chrono::microseconds>(t4 - ts3).count() / 1000.0
                                 << " ms");

    // action from NN
    at::Tensor action = out.index({Slice(0, COMMAND_SIZE)});

    // reconstructed hidden information
    at::Tensor hidden = out.index({Slice(COMMAND_SIZE, None)});

    // unpack action
    at::Tensor phaseOffsets = action.index({Slice(0, 4)});
    vector_t phaseOffsetsVec(4);
    for (size_t i = 0; i < 4; ++i) {
        phaseOffsetsVec[i] = (phaseOffsets[i].item<float>() * ACTION_SCALE);
    }

    // std::cout << phaseOffsetsVec.transpose() << std::endl << std::endl;
    at::Tensor jointResiduals = action.index({Slice(4, COMMAND_SIZE)});
    vector_t jointResidualsVec(12);
    for (size_t i = 0; i < 12; ++i) {
        jointResidualsVec[i] = (jointResiduals[i].item<float>() * ACTION_SCALE);
    }

    // compute ik
    auto legHeights = cpg_->legHeights(phaseOffsetsVec);
    jointAngles2_ = ik_->solve(legHeights);
    for (int i = 0; i < 12; ++i) {
        jointAngles2_[i] += jointResidualsVec[i];
    }


    // generate command message
    bobnet_msgs::JointCommandArray commandMessage;
    commandMessage.joint_commands.resize(jointNames_.size());
    for (size_t i = 0; i < jointNames_.size(); ++i) {
        commandMessage.joint_commands[i].joint_name = jointNames_[i];
        commandMessage.joint_commands[i].position_desired = jointAngles2_[i];
        commandMessage.joint_commands[i].velocity_desired = 0.0;
        commandMessage.joint_commands[i].kp = kp_;
        commandMessage.joint_commands[i].kd = kd_;
        commandMessage.joint_commands[i].torque_ff = 0.0;
    }

    legHeights = cpg_->legHeights();
    jointAngles2_ = ik_->solve(legHeights);
    for (int i = 0; i < 12; ++i) {
        jointAngles2_[i] += jointResidualsVec[i];
    }

    // update history buffer
    updateHistory(nnInput, action);
    // update central pattern generator
    cpg_->step(dt);

    // visualize
    visualizer_.visualize(state, sampled_, hidden.index({samplesReconstructedSlice_}));

    return commandMessage;
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
at::Tensor RlController::getNNInput(const State &state, scalar_t dt) {
    at::Tensor input = at::empty(getNNInputSize());

    // Fill individual sections of the nn input tensor
    fillCommand(input, dt);
    fillGravity(input, state);
    fillBaseLinearVelocity(input, state);
    fillBaseAngularVelocity(input, state);
    fillJointResiduals(input, state);
    fillJointVelocities(input, state);
    fillHistory(input);
    fillCpg(input);
    fillHeights(input, state);
    return input;
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void RlController::fillCommand(at::Tensor &input, scalar_t dt) {
    auto command = refGen_->getVelocityReference(dt);
    input[0] = command.velocity_x;
    input[1] = command.velocity_y;
    input[2] = command.yaw_rate;
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void RlController::fillGravity(at::Tensor &input, const State &state) {
    input[3] = state.normalizedGravityBase[0] * GRAVITY_SCALE;
    input[4] = state.normalizedGravityBase[1] * GRAVITY_SCALE;
    input[5] = state.normalizedGravityBase[2] * GRAVITY_SCALE;
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void RlController::fillBaseLinearVelocity(at::Tensor &input, const State &state) {
    input[6] = state.baseLinearVelocityBase[0] * LIN_VEL_SCALE;
    input[7] = state.baseLinearVelocityBase[1] * LIN_VEL_SCALE;
    input[8] = state.baseLinearVelocityBase[2] * LIN_VEL_SCALE;
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void RlController::fillBaseAngularVelocity(at::Tensor &input, const State &state) {
    input[9] = state.baseAngularVelocityBase[0] * ANG_VEL_SCALE;
    input[10] = state.baseAngularVelocityBase[1] * ANG_VEL_SCALE;
    input[11] = state.baseAngularVelocityBase[2] * ANG_VEL_SCALE;
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void RlController::fillJointResiduals(at::Tensor &input, const State &state) {
    // fill joint residuals
    for (size_t i = 0; i < 12; ++i) {
        input[12 + i] = (state.jointPositions[i] - jointAngles2_[i]) * JOINT_POS_SCALE;
    }
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void RlController::fillJointVelocities(at::Tensor &input, const State &state) {
    // fill joint velocities
    for (size_t i = 0; i < 12; ++i) {
        input[24 + i] = state.jointVelocities[i] * JOINT_VEL_SCALE;
    }
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void RlController::fillHistory(at::Tensor &input) {
    fillHistoryResiduals(input);
    fillHistoryVelocities(input);
    fillHistoryActions(input);
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void RlController::fillCpg(at::Tensor &input) {
    const size_t startIdx = 36 + POSITION_HISTORY_SIZE * 12 + VELOCITY_HISTORY_SIZE * 12 + COMMAND_HISTORY_SIZE * 16;
    auto cpgObservation = cpg_->getObservation();
    for (size_t i = 0; i < 8; ++i) {
        input[startIdx + i] = cpgObservation[i];
    }
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void RlController::fillHeights(at::Tensor &input, const State &state) {
    const size_t startIdx =
        36 + POSITION_HISTORY_SIZE * 12 + VELOCITY_HISTORY_SIZE * 12 + COMMAND_HISTORY_SIZE * 16 + 8;

    // Find yaw angle
    Eigen::Quaterniond q(state.baseOrientationWorld[3], state.baseOrientationWorld[0], state.baseOrientationWorld[1],
                         state.baseOrientationWorld[2]);

    double yaw = q.toRotationMatrix().eulerAngles(2, 1, 0)[0];

    // Rotate sampling points
    Eigen::Matrix3d Ryaw = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Eigen::MatrixXd rotatedSamplingPoints = Ryaw * gridmap_->samplingPositions_;

    // Replicate sampling points
    sampled_ = rotatedSamplingPoints.replicate<1, 4>();

    auto addOffset = [this](scalar_t x_offset, scalar_t y_offset, size_t idx) mutable {
        size_t blockStart = idx * 52;
        sampled_.block<1, 52>(0, blockStart).array() += x_offset;
        sampled_.block<1, 52>(1, blockStart).array() += y_offset;
    };
    addOffset(state.lfFootPositionWorld[0], state.lfFootPositionWorld[1], 0);
    addOffset(state.lhFootPositionWorld[0], state.lhFootPositionWorld[1], 1);
    addOffset(state.rfFootPositionWorld[0], state.rfFootPositionWorld[1], 2);
    addOffset(state.rhFootPositionWorld[0], state.rhFootPositionWorld[1], 3);

    // sample heights
    gridmap_->atPositions(sampled_);

    // clamp third row between -1 and 1
    // sampled_.row(2) = (state.basePositionWorld[2] - sampled_.row(2).array()).array() - 0.5;
    // sampled_.row(2) = sampled_.row(2).cwiseMax(-1.0).cwiseMin(1.0) * HEIGHT_MEASUREMENTS_SCALE;

    // auto options = torch::TensorOptions().dtype(torch::kDouble);
    // input.index({Slice(startIdx, None)}) = torch::from_blob(sampled_.row(2).data(), {4 * 52}, options);

    for(int i = 0; i < 4*52; ++i) {
        float out = state.basePositionWorld[2] - 0.5 - sampled_(2, i);
        if (out > 1.0) {
            out = 1.0;
        } else if (out < -1.0) {
            out = -1.0;
        }

        input[startIdx + i] = out * HEIGHT_MEASUREMENTS_SCALE;
        sampled_(2, i) = out;
    }
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void RlController::fillHistoryResiduals(at::Tensor &input) {
    int ip = mod(historyResidualsIndex_ - 1, POSITION_HISTORY_SIZE);
    for (int i = 0; i < POSITION_HISTORY_SIZE; ++i) {
        int idx = mod(ip + i, POSITION_HISTORY_SIZE);
        input.index({Slice(36 + i * 12, 36 + (i + 1) * 12)}) = historyResiduals_[idx];
    }
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void RlController::fillHistoryVelocities(at::Tensor &input) {
    int ip = mod(historyVelocitiesIndex_ - 1, VELOCITY_HISTORY_SIZE);
    for (int i = 0; i < VELOCITY_HISTORY_SIZE; ++i) {
        int idx = mod(ip + i, VELOCITY_HISTORY_SIZE);
        input.index({Slice(36 + POSITION_HISTORY_SIZE * 12 + i * 12, 36 + POSITION_HISTORY_SIZE * 12 + (i + 1) * 12)}) =
            historyVelocities_[idx];
    }
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void RlController::fillHistoryActions(at::Tensor &input) {
    int ip = mod(historyActionsIndex_ - 1, COMMAND_HISTORY_SIZE);
    for (int i = 0; i < COMMAND_HISTORY_SIZE; ++i) {
        int idx = mod(ip + i, COMMAND_HISTORY_SIZE);
        input.index({Slice(36 + POSITION_HISTORY_SIZE * 12 + VELOCITY_HISTORY_SIZE * 12 + i * 16,
                           36 + POSITION_HISTORY_SIZE * 12 + VELOCITY_HISTORY_SIZE * 12 + (i + 1) * 16)}) =
            historyActions_[idx];
    }
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/

void RlController::updateHistory(const at::Tensor &input, const at::Tensor &action) {
    // update position history
    historyResiduals_[historyResidualsIndex_] = input.index({jointResidualsSlice_});
    historyResidualsIndex_ = (historyResidualsIndex_ + 1) % POSITION_HISTORY_SIZE;

    // update velocity history
    historyVelocities_[historyVelocitiesIndex_] = input.index({jointVelocitiesSlice_});
    historyVelocitiesIndex_ = (historyVelocitiesIndex_ + 1) % VELOCITY_HISTORY_SIZE;

    // update action history
    historyActions_[historyActionsIndex_] = action;
    historyActionsIndex_ = (historyActionsIndex_ + 1) % COMMAND_HISTORY_SIZE;
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void RlController::resetHistory() {
    auto reset = [](auto &history, size_t history_size, size_t item_size) {
        history.clear();
        for (size_t i = 0; i < history_size; ++i) {
            history.push_back(torch::zeros({item_size}));
        }
    };
    reset(historyResiduals_, POSITION_HISTORY_SIZE, POSITION_SIZE);
    reset(historyVelocities_, VELOCITY_HISTORY_SIZE, VELOCITY_SIZE);
    reset(historyActions_, COMMAND_HISTORY_SIZE, COMMAND_SIZE);
}

}  // namespace bobnet_controllers