#include <ros/ros.h>

#include <bobnet_config/utils.h>
#include <bobnet_core/Types.h>
#include <bobnet_control/CentralController.h>

using namespace bobnet_core;

int main(int argc, char *argv[]) {
    ROS_INFO("Starting bobnet_control node");
    ros::init(argc, argv, "bobnet_control");

    bobnet_control::CentralController controller;
    scalar_t rate = bobnet_config::fromRosConfigFile<scalar_t>("rate");
    ros::Rate loopRate(rate);


    ros::Time lastTime = ros::Time::now();
    while (ros::ok()) {
        ros::spinOnce();

        ros::Time currentTime = ros::Time::now();
        controller.step((currentTime - lastTime).toSec());
        lastTime = currentTime;

        loopRate.sleep();
    }

}