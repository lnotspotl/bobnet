#pragma once

#include <bobnet_msgs/JointCommandArray.h>

#include <memory>

#include <bobnet_controllers/Types.h>
#include <bobnet_controllers/InverseKinematics.h>
#include <bobnet_controllers/CentralPatternGenerator.h>

namespace bobnet_controllers {

enum class ControllerType { STAND, RL };

class Controller {
   public:
    virtual bobnet_msgs::JointCommandArray getCommandMessage() = 0;
    virtual ~Controller() = default;
};

class StandController : public Controller {
   public:
    StandController(std::vector<std::string> jointNames, std::vector<double> jointAngles, scalar_t kp, scalar_t kd);
    bobnet_msgs::JointCommandArray getCommandMessage() override;

   private:
    std::vector<std::string> jointNames_;
    std::vector<double> jointAngles_;
    scalar_t kp_;
    scalar_t kd_;
};

class RlController : public Controller {
   public:
    RlController(std::vector<std::string> jointNames, scalar_t kp, scalar_t kd, std::unique_ptr<InverseKinematics> ik,
                 std::unique_ptr<CentralPatternGenerator> cpg) {
        jointNames_ = jointNames;
        kp_ = kp;
        kd_ = kd;
        ik_ = std::move(ik);
        cpg_ = std::move(cpg);
    }

    bobnet_msgs::JointCommandArray getCommandMessage() override;

   private:
    scalar_t kp_;
    scalar_t kd_;

    std::unique_ptr<InverseKinematics> ik_;
    std::unique_ptr<CentralPatternGenerator> cpg_;

    std::vector<std::string> jointNames_;
};

}  // namespace bobnet_controllers