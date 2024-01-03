#pragma once

#include <memory>
#include <bobnet_control/JointPID.h>
#include <bobnet_control/StateSubscriber.h>
#include <bobnet_control/Controller.h>
#include <bobnet_core/Types.h>

namespace bobnet_control {

class StaticController : public Controller {
   public:
    StaticController(std::shared_ptr<JointPID> &pidControllerPtr, std::shared_ptr<StateSubscriber> &stateSubscriberPtr);

    void sendCommand(const scalar_t dt) override;

    void visualize() override;

    void changeController(const std::string &controllerType) override;

    bool isSupported(const std::string &controllerType) override;

   private:
    scalar_t kp_;
    scalar_t kd_;

    void publishStand();
    void publishSit();
    void publishInterp(const scalar_t dt);

    std::shared_ptr<JointPID> pidControllerPtr_;
    std::shared_ptr<StateSubscriber> stateSubscriberPtr_;

    vector_t sitJointAngles_;
    vector_t standJointAngles_;

    scalar_t alpha_;
    vector_t interpFrom_;
    vector_t interpTo_;

    std::string controllerType_;

    scalar_t interpolationTime_;

    std::vector<std::string> jointNames_;
};

}  // namespace bobnet_control