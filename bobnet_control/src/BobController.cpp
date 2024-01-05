#include <pinocchio/fwd.hpp>
#include "bobnet_control/BobController.h"

#include <bobnet_config/utils.h>

#include <ros/ros.h>

#include <chrono>
#include <pinocchio/math/rpy.hpp>

namespace bobnet_control {

inline int mod(int a, int b) { return (a % b + b) % b; }

BobController::BobController(std::shared_ptr<JointPID> &pidControllerPtr,
                             std::shared_ptr<StateSubscriber> &stateSubscriberPtr)
    : pidControllerPtr_(pidControllerPtr), stateSubscriberPtr_(stateSubscriberPtr) {
    kp_ = bobnet_config::fromRosConfigFile<scalar_t>("bob_controller/kp");
    kd_ = bobnet_config::fromRosConfigFile<scalar_t>("bob_controller/kd");

    ROS_INFO("Getting IK");
    ik_ = bobnet_control::getInverseKinematicsUnique();

    ROS_INFO("Getting CPG");
    cpg_ = bobnet_control::getCentralPatternGeneratorUnique();

    ROS_INFO("Getting refGen");
    refGen_ = bobnet_reference::getReferenceGeneratorUnique();

    ROS_INFO("Getting gridmap");
    gridmap_ = bobnet_gridmap::getGridmapInterfaceUnique();

    auto modelPath = bobnet_config::fromRosConfigFile<std::string>("bob_controller/model_path");
    ROS_INFO_STREAM("[BobController] Loading model from: " << modelPath);

    try {
        model_ = torch::jit::load(modelPath);
    } catch (const c10::Error &e) {
        std::cerr << "Could not load model from: " << modelPath << std::endl;
        throw std::runtime_error("Could not load model");
    }

    std::vector<torch::jit::IValue> stack;
    model_.get_method("set_hidden_size")(stack);
    resetHistory();

    // auto legHeights = cpg_->legHeights();
    // jointAngles2_ = ik_->solve(legHeights);

    standJointAngles_ = bobnet_config::fromRosConfigFile<vector_t>("static_controller/stand_controller/joint_angles");
}

void BobController::visualize() { visualizer_.visualize(stateSubscriberPtr_->getState(), sampled_, hidden_); }

void BobController::changeController(const std::string &controllerType) {}

bool BobController::isSupported(const std::string &controllerType) {
    if (controllerType == "BOB" || controllerType == "RL") {
        return true;
    }
    return false;
}

void BobController::updateHistory(const at::Tensor &input, const at::Tensor &action) {
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
void BobController::resetHistory() {
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

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void BobController::fillCommand(at::Tensor &input, scalar_t dt) {
    auto command = refGen_->getVelocityReference(dt);
    input[0] = command.velocity_x;
    input[1] = command.velocity_y;
    input[2] = command.yaw_rate;
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void BobController::fillGravity(at::Tensor &input, const State &state) {
    input[3] = state.normalizedGravityBase[0] * GRAVITY_SCALE;
    input[4] = state.normalizedGravityBase[1] * GRAVITY_SCALE;
    input[5] = state.normalizedGravityBase[2] * GRAVITY_SCALE;
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void BobController::fillBaseLinearVelocity(at::Tensor &input, const State &state) {
    input[6] = state.baseLinearVelocityBase[0] * LIN_VEL_SCALE;
    input[7] = state.baseLinearVelocityBase[1] * LIN_VEL_SCALE;
    input[8] = state.baseLinearVelocityBase[2] * LIN_VEL_SCALE;
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void BobController::fillBaseAngularVelocity(at::Tensor &input, const State &state) {
    input[9] = state.baseAngularVelocityBase[0] * ANG_VEL_SCALE;
    input[10] = state.baseAngularVelocityBase[1] * ANG_VEL_SCALE;
    input[11] = state.baseAngularVelocityBase[2] * ANG_VEL_SCALE;
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void BobController::fillJointResiduals(at::Tensor &input, const State &state) {
    // fill joint residuals
    for (size_t i = 0; i < 12; ++i) {
        input[12 + i] = (state.jointPositions[i] - standJointAngles_[i]) * JOINT_POS_SCALE;
    }
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void BobController::fillJointVelocities(at::Tensor &input, const State &state) {
    // fill joint velocities
    for (size_t i = 0; i < 12; ++i) {
        input[24 + i] = state.jointVelocities[i] * JOINT_VEL_SCALE;
    }
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void BobController::fillHistory(at::Tensor &input) {
    fillHistoryResiduals(input);
    fillHistoryVelocities(input);
    fillHistoryActions(input);
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void BobController::fillCpg(at::Tensor &input) {
    const size_t startIdx = 36 + POSITION_HISTORY_SIZE * 12 + VELOCITY_HISTORY_SIZE * 12 + COMMAND_HISTORY_SIZE * 16;
    auto cpgObservation = cpg_->getObservation();
    for (size_t i = 0; i < 8; ++i) {
        input[startIdx + i] = cpgObservation[i];
    }
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void BobController::fillHeights(at::Tensor &input, const State &state) {
    const size_t startIdx =
        36 + POSITION_HISTORY_SIZE * 12 + VELOCITY_HISTORY_SIZE * 12 + COMMAND_HISTORY_SIZE * 16 + 8;

    // Find yaw angle
    quaternion_t q(state.baseOrientationWorld[3], state.baseOrientationWorld[0], state.baseOrientationWorld[1],
                   state.baseOrientationWorld[2]);

    scalar_t yaw = pinocchio::rpy::matrixToRpy(q.toRotationMatrix())[2];

    // Rotate sampling points
    matrix3_t Ryaw = angleaxis_t(yaw, vector3_t::UnitZ()).toRotationMatrix();
    matrix_t rotatedSamplingPoints = Ryaw * gridmap_->samplingPositions_;

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
    sampled_.row(2) = (state.basePositionWorld[2] - sampled_.row(2).array()).array() - 0.5;
    sampled_.row(2) = sampled_.row(2).cwiseMax(-1.0).cwiseMin(1.0) * HEIGHT_MEASUREMENTS_SCALE;

    // Eigen -> torch: https://discuss.pytorch.org/t/data-transfer-between-libtorch-c-and-eigen/54156/6
    float *torchPtr = input.index({Slice(startIdx, None)}).data_ptr<float>();
    Eigen::Map<Eigen::VectorXf> ef(torchPtr, 4 * 52, 1);
    ef = sampled_.row(2).cast<float>();
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void BobController::fillHistoryResiduals(at::Tensor &input) {
    int ip = mod(historyResidualsIndex_ - 1, POSITION_HISTORY_SIZE);
    for (int i = 0; i < POSITION_HISTORY_SIZE; ++i) {
        int idx = mod(ip + i, POSITION_HISTORY_SIZE);
        input.index({Slice(36 + i * 12, 36 + (i + 1) * 12)}) = historyResiduals_[idx];
    }
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void BobController::fillHistoryVelocities(at::Tensor &input) {
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
void BobController::fillHistoryActions(at::Tensor &input) {
    int ip = mod(historyActionsIndex_ - 1, COMMAND_HISTORY_SIZE);
    for (int i = 0; i < COMMAND_HISTORY_SIZE; ++i) {
        int idx = mod(ip + i, COMMAND_HISTORY_SIZE);
        input.index({Slice(36 + POSITION_HISTORY_SIZE * 12 + VELOCITY_HISTORY_SIZE * 12 + i * 16,
                           36 + POSITION_HISTORY_SIZE * 12 + VELOCITY_HISTORY_SIZE * 12 + (i + 1) * 16)}) =
            historyActions_[idx];
    }
}

at::Tensor BobController::getNNInput(const State &state, scalar_t dt) {
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

void BobController::sendCommand(const scalar_t dt) {
    auto &state = stateSubscriberPtr_->getState();

    // Do not keep track of gradients
    torch::NoGradGuard no_grad;

    auto ts1 = std::chrono::high_resolution_clock::now();
    at::Tensor nnInput = getNNInput(state, dt);
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
    hidden_ = out.index({Slice(COMMAND_SIZE, None)});

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
    jointAngles2_ = ik_->solve(legHeights) + jointResidualsVec;

    pidControllerPtr_->sendCommand(jointAngles2_, kp_, kd_);

    // legHeights = cpg_->legHeights();
    // jointAngles2_ = ik_->solve(legHeights) + jointResidualsVec;

    // update history buffer
    updateHistory(nnInput, action);
    // update central pattern generator
    cpg_->step(dt);
}

}  // namespace bobnet_control