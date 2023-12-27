#include <memory>
#include <string>

#include <ros/ros.h>

#include <bobnet_controllers/Controllers.h>
#include <bobnet_controllers/CentralPatternGenerator.h>

#include <bobnet_controllers/anymal_c/AnymalCInfo.h>
#include <bobnet_controllers/anymal_c/AnymalCInverseKinematics.h>

int main(int argc, char *argv[]) {
    ROS_INFO("Starting bobnet_controllers node");
    ros::init(argc, argv, "bobnet_controllers");
    ros::NodeHandle nh;

    anymal_c::AnymalCInfo info;
    auto ik = std::unique_ptr<bobnet_controllers::InverseKinematics>(new anymal_c::AnymalCInverseKinematics());
    auto cpg = std::unique_ptr<bobnet_controllers::CentralPatternGenerator>(
        new bobnet_controllers::CentralPatternGenerator(info.cpgPeriod, info.swingHeight, info.cpgPhaseOffsets));

    ROS_INFO("Setup done, starting controller");

    const std::string jointCommandTopic = "/anymal_c/joint_controller/bobnet_gazebo/joint_controller/command";
    ros::Publisher publisher = nh.advertise<bobnet_msgs::JointCommandArray>(jointCommandTopic, 1);

    bobnet_controllers::StandController controller(info.jointNames, info.standControllerJointAngles, 400.0, 10.0);

    ros::Rate rate(50);
    while (ros::ok()) {
        auto commandMsg = controller.getCommandMessage();
        publisher.publish(commandMsg);
        rate.sleep();
    }

    return EXIT_SUCCESS;
}