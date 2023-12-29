#include <bobnet_visualization/RvizVisualizer.h>

#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>

#include <geometry_msgs/TransformStamped.h>

namespace bobnet_visualization {

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
RvizVisualizer::RvizVisualizer() {
    ros::NodeHandle nh;

    std::string urdfString;
    if(!nh.getParam("/robot_description", urdfString)) {
        ROS_ERROR("Failed to get param /robot_description");
        return;
    }
    
    KDL::Tree kdlTree;
    kdl_parser::treeFromString(urdfString, kdlTree);

    robotStatePublisherPtr_.reset(new robot_state_publisher::RobotStatePublisher(kdlTree));
    robotStatePublisherPtr_->publishFixedTransforms(true);

    worldFrame_ = "odom";
}

}  // namespace bobnet_visualization