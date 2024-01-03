#pragma once

#include <string>
#include <vector>
#include <memory>

#include <Eigen/Dense>

#include <bobnet_core/Types.h>
#include <bobnet_control/Controller.h>
#include <bobnet_control/JointPID.h>
#include <bobnet_control/StateSubscriber.h>

#include <bobnet_control/InverseKinematics.h>
#include <bobnet_control/CentralPatternGenerator.h>
#include <bobnet_reference/ReferenceGenerator.h>
#include <bobnet_gridmap/GridmapInterface.h>

#include <bobnet_visualization/Visualizers.h>

#include <torch/script.h>

namespace bobnet_control {

using namespace bobnet_core;
using namespace torch::indexing;
using torch::jit::script::Module;

class BobController : public Controller {
   public:
    BobController(std::shared_ptr<JointPID> &pidControllerPtr, std::shared_ptr<StateSubscriber> &stateSubscriberPtr);

    void sendCommand(const scalar_t dt) override;

    void visualize() override;

    void changeController(const std::string &controllerType) override;

    bool isSupported(const std::string &controllerType) override;

   private:
    std::shared_ptr<JointPID> pidControllerPtr_;
    std::shared_ptr<StateSubscriber> stateSubscriberPtr_;

    scalar_t kp_;
    scalar_t kd_;

    std::unique_ptr<InverseKinematics> ik_;
    std::unique_ptr<CentralPatternGenerator> cpg_;
    std::unique_ptr<bobnet_reference::ReferenceGenerator> refGen_;
    std::unique_ptr<bobnet_gridmap::GridmapInterface> gridmap_;

    /* Copied section */

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

    bobnet_visualization::HeightsReconstructedVisualizer visualizer_;

    Module model_;

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
    int historyActionsIndex_ = 0;

    // at::Tensor nnInput_;
    at::Tensor hidden_;

    vector_t jointAngles2_;

    matrix_t sampled_;

};

}  // namespace bobnet_control