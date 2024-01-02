#include <memory>
#include <string>


#include <ros/ros.h>
#include <bobnet_control/RobotInterface.h>
#include <bobnet_control/Controllers.h>
#include <bobnet_control/CentralPatternGenerator.h>
#include <bobnet_gridmap/GridmapInterface.h>

#include <bobnet_control/anymal_c/AnymalCInfo.h>
#include <bobnet_control/anymal_c/AnymalCInverseKinematics.h>

using bobnet_control::scalar_t;

int main(int argc, char *argv[]) {
    ROS_INFO("Starting bobnet_control node");
    ros::init(argc, argv, "bobnet_control");
    ros::NodeHandle nh;

    anymal_c::AnymalCInfo info;
    auto ik = std::unique_ptr<bobnet_control::InverseKinematics>(new anymal_c::AnymalCInverseKinematics());
    auto cpg = std::unique_ptr<bobnet_control::CentralPatternGenerator>(
        new bobnet_control::CentralPatternGenerator(info.cpgPeriod, info.swingHeight, info.cpgPhaseOffsets));

    ROS_INFO("Setup done, starting controller");

    auto standControllerPtr = std::unique_ptr<bobnet_control::StandController>(
        new bobnet_control::StandController(info.jointNames, info.standControllerJointAngles, 400, 10));

    const std::string stateTopic = "/anymal_c/state";
    const std::string changeControllerTopic = "/anymal_c/changeController";
    const std::string commandTopic = "anymal_c/command";

    scalar_t rate = 50;
    if (!nh.getParam("/bobnet/interface_rate", rate)) {
        ROS_WARN_STREAM("Could not get rate parameter, using default value: " << rate);
    }

    auto refPtr = std::unique_ptr<bobnet_reference::JoystickReferenceGenerator>(
        new bobnet_reference::JoystickReferenceGenerator(nh, "/joy", 10.5, 1, 0, 3, 1.0, 1.0, 1.0));

    std::string rlModelPath = "";
    if (!nh.getParam("/rl_policy_path", rlModelPath)) {
        ROS_WARN_STREAM("Could not get rl_model_path parameter, using default value: " << rlModelPath);
    }

    auto gridmapPtr = std::unique_ptr<bobnet_gridmap::GridmapInterface>(
        new bobnet_gridmap::GridmapInterface(nh, "/anymal_c/grid_map"));
    gridmapPtr->waitTillInitialized();

    auto rlControllerPtr = std::unique_ptr<bobnet_control::RlController>(new bobnet_control::RlController(
        info.jointNames, 80, 2, std::move(ik), std::move(cpg), std::move(refPtr), std::move(gridmapPtr), rlModelPath));

    bobnet_control::RobotInterface interface(stateTopic, changeControllerTopic, commandTopic, rate,
                                                 std::move(standControllerPtr), std::move(rlControllerPtr));

    interface.setup();
    interface.run();

    return EXIT_SUCCESS;
}