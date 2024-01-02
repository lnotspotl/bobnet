#pragma once

#include <bobnet_msgs/JointCommandArray.h>

#include <ros/ros.h>

#include <memory>

#include <bobnet_core/State.h>
#include <bobnet_core/Types.h>
#include <bobnet_controllers/InverseKinematics.h>
#include <bobnet_controllers/CentralPatternGenerator.h>
#include <bobnet_reference/ReferenceGenerator.h>
#include <bobnet_gridmap/GridmapInterface.h>
#include <bobnet_controllers/Utils.h>
#include <bobnet_visualization/anymal_c/AnymalCVisualizer.h>

#include <Eigen/Dense>
#include <bobnet_msgs/RobotState.h>

#include <torch/script.h>

namespace bobnet_controllers {

using namespace bobnet_core;

enum class ControllerType { STAND, RL, NUM_CONTROLLER_TYPES };

std::string controllerType2String(ControllerType type);
ControllerType string2ControllerType(const std::string &type);

class Controller {
   public:
    virtual bobnet_msgs::JointCommandArray getCommandMessage(const State &state, scalar_t dt) = 0;
    virtual ~Controller() = default;
};

class StandController : public Controller {
   public:
    StandController(std::vector<std::string> jointNames, vector_t jointAngles, scalar_t kp, scalar_t kd);
    bobnet_msgs::JointCommandArray getCommandMessage(const State &state, scalar_t dt) override;

   private:
    std::vector<std::string> jointNames_;
    vector_t jointAngles_;
    scalar_t kp_;
    scalar_t kd_;
};

using namespace torch::indexing;
using torch::jit::script::Module;
class RlController : public Controller {
   public:
    RlController(std::vector<std::string> jointNames, scalar_t kp, scalar_t kd, std::unique_ptr<InverseKinematics> ik,
                 std::unique_ptr<CentralPatternGenerator> cpg,
                 std::unique_ptr<bobnet_reference::ReferenceGenerator> refGen,
                 std::unique_ptr<bobnet_gridmap::GridmapInterface> gridmap, const std::string &modelPath) {
        jointNames_ = jointNames;
        kp_ = kp;
        kd_ = kd;
        ik_ = std::move(ik);
        cpg_ = std::move(cpg);
        refGen_ = std::move(refGen);
        gridmap_ = std::move(gridmap);

        resetHistory();
        model_ = loadTorchModel(modelPath);

        // TODO: put this into the loadTorchModel method
        std::vector<torch::jit::IValue> stack;
        model_.get_method("set_hidden_size")(stack);

        auto legHeights = cpg_->legHeights();
        jointAngles2_ = ik_->solve(legHeights);
    }

    bobnet_msgs::JointCommandArray getCommandMessage(const State &state, scalar_t dt) override;

   private:
    scalar_t kp_;
    scalar_t kd_;

    std::unique_ptr<InverseKinematics> ik_;
    std::unique_ptr<CentralPatternGenerator> cpg_;
    std::unique_ptr<bobnet_reference::ReferenceGenerator> refGen_;
    std::unique_ptr<bobnet_gridmap::GridmapInterface> gridmap_;

    scalar_t LIN_VEL_SCALE = 2.0;
    scalar_t ANG_VEL_SCALE = 0.25;
    scalar_t GRAVITY_SCALE = 1.0;
    scalar_t COMMAND_SCALE = 1.0;
    scalar_t JOINT_POS_SCALE = 1.0;
    scalar_t JOINT_VEL_SCALE = 0.05;
    scalar_t ACTION_SCALE = 0.5;
    scalar_t HEIGHT_MEASUREMENTS_SCALE = 1.0;

    int POSITION_HISTORY_SIZE = 3;
    int VELOCITY_HISTORY_SIZE = 2;
    int COMMAND_HISTORY_SIZE = 2;

    int POSITION_SIZE = 12;
    int VELOCITY_SIZE = 12;
    int COMMAND_SIZE = 16;

    bobnet_visualization::AnymalCVisualizer visualizer_;

    Module model_;

    std::vector<std::string> jointNames_;

    constexpr size_t getNNInputSize() { return 3 + 3 + 3 + 3 + 12 + 12 + 3 * 12 + 2 * 12 + 2 * 16 + 8 + 4 * 52; }

    at::Tensor getNNInput(const State &state, scalar_t dt);

    void fillCommand(at::Tensor &input, scalar_t dt);
    void fillGravity(at::Tensor &input, const State &state);
    void fillBaseLinearVelocity(at::Tensor &input, const State &state);
    void fillBaseAngularVelocity(at::Tensor &input, const State &state);
    void fillJointResiduals(at::Tensor &input, const State &state);
    void fillJointVelocities(at::Tensor &input, const State &state);
    void fillHistory(at::Tensor &input);
    void fillCpg(at::Tensor &input);
    void fillHeights(at::Tensor &input, const State &state);

    void fillHistoryResiduals(at::Tensor &input);
    void fillHistoryVelocities(at::Tensor &input);
    void fillHistoryActions(at::Tensor &input);

    void updateHistory(const at::Tensor &input, const at::Tensor &action);
    void resetHistory();

    const Slice commandSlice_ = Slice(0, 3);
    const Slice gravitySlice_ = Slice(3, 6);
    const Slice baseLinearVelocitySlice_ = Slice(6, 9);
    const Slice baseAngularVelocitySlice_ = Slice(9, 12);
    const Slice jointResidualsSlice_ = Slice(12, 24);
    const Slice jointVelocitiesSlice_ = Slice(24, 36);
    const Slice samplesGTSlice_ = Slice(136, None);
    const Slice samplesReconstructedSlice_ = Slice(0, 4*52);

    std::vector<at::Tensor> historyResiduals_;
    int historyResidualsIndex_ = 0;
    std::vector<at::Tensor> historyVelocities_;
    int historyVelocitiesIndex_ = 0;
    std::vector<at::Tensor> historyActions_;
    int historyActionsIndex_ = 1;

    vector_t jointAngles2_;

    Eigen::MatrixXd sampled_;
};

}  // namespace bobnet_controllers