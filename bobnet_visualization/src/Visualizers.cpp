#include "bobnet_visualization/Visualizers.h"

#include <bobnet_config/utils.h>

#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <geometry_msgs/TransformStamped.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


namespace bobnet_visualization {

StateVisualizer::StateVisualizer() {
    // Load odom frame name
    odomFrame_ = bobnet_config::fromRosConfigFile<std::string>("odom_frame");

    // Load base frame name
    baseFrame_ = bobnet_config::fromRosConfigFile<std::string>("base_name");

    // Load joint names
    jointNames_ = bobnet_config::fromRosConfigFile<std::vector<std::string>>("joint_names");

    // Setup state publisher
    std::string urdfString;
    if (!ros::param::get("/robot_description", urdfString)) {
        throw std::runtime_error("Failed to get param /robot_description");
    }

    KDL::Tree kdlTree;
    kdl_parser::treeFromString(urdfString, kdlTree);
    robotStatePublisherPtr_.reset(new robot_state_publisher::RobotStatePublisher(kdlTree));
    robotStatePublisherPtr_->publishFixedTransforms(true);

    ROS_INFO("StateVisualizer initialized");
}

void StateVisualizer::visualize(const bobnet_core::State &state) {
    ros::Time timeStamp = ros::Time::now();
    publishOdomTransform(timeStamp, state);
    publishJointAngles(timeStamp, state);
}

void StateVisualizer::publishOdomTransform(const ros::Time &timeStamp, const State &state) {
    geometry_msgs::TransformStamped baseToWorldTransform;
    baseToWorldTransform.header.stamp = timeStamp;
    baseToWorldTransform.header.frame_id = odomFrame_;
    baseToWorldTransform.child_frame_id = baseFrame_;

    baseToWorldTransform.transform.translation.x = state.basePositionWorld[0];
    baseToWorldTransform.transform.translation.y = state.basePositionWorld[1];
    baseToWorldTransform.transform.translation.z = state.basePositionWorld[2];

    baseToWorldTransform.transform.rotation.x = state.baseOrientationWorld[0];
    baseToWorldTransform.transform.rotation.y = state.baseOrientationWorld[1];
    baseToWorldTransform.transform.rotation.z = state.baseOrientationWorld[2];
    baseToWorldTransform.transform.rotation.w = state.baseOrientationWorld[3];

    tfBroadcaster_.sendTransform(baseToWorldTransform);
}

void StateVisualizer::publishJointAngles(const ros::Time &timeStamp, const State &state) {
    std::map<std::string, double> positions;
    for (int i = 0; i < jointNames_.size(); ++i) {
        positions[jointNames_[i]] = state.jointPositions[i];
    }
    robotStatePublisherPtr_->publishTransforms(positions, timeStamp);
}

HeightsReconstructedVisualizer::HeightsReconstructedVisualizer() {
    ros::NodeHandle nh;
    std::string markerTopic = bobnet_config::fromRosConfigFile<std::string>("marker_topic");
    markerPublisher_ = nh.advertise<visualization_msgs::MarkerArray>(markerTopic, 1);

    odomFrame_ = bobnet_config::fromRosConfigFile<std::string>("odom_frame");
}

void HeightsReconstructedVisualizer::visualize(const State &state, const matrix_t &sampled,
                                               const at::Tensor &nnPointsReconstructed) {
    ros::Time timeStamp = ros::Time::now();

    publishMarkers(timeStamp, sampled, {1.0, 0.0, 0.0}, "ground_truth",
                   [&](size_t id) { return -(sampled(2, id) / 1.0 + 0.5 - state.basePositionWorld[2]); });

    publishMarkers(timeStamp, sampled, {0.0, 0.0, 1.0}, "nn_reconstructed", [&](size_t id) {
        float out = -(nnPointsReconstructed[id].item<float>() / 1.0 + 0.5 - state.basePositionWorld[2]);
        return static_cast<scalar_t>(out);
    });
}

void HeightsReconstructedVisualizer::publishMarkers(const ros::Time &timeStamp, const matrix_t &sampled,
                                                    const std::array<float, 3> &rgb, const std::string &markerNamePrefix,
                                                    std::function<scalar_t(size_t)> heightFunction) {
    const size_t nPoints = sampled.cols();
    const size_t pointsPerLeg = nPoints / 4;

    constexpr uint32_t shape = visualization_msgs::Marker::SPHERE;
    visualization_msgs::MarkerArray markerArray;
    markerArray.markers.reserve(nPoints);

    for (size_t leg_idx = 0; leg_idx < 4; ++leg_idx) {
        const std::string ns = markerNamePrefix + std::to_string(leg_idx);
        for (size_t i = 0; i < pointsPerLeg; ++i) {
            size_t id = leg_idx * pointsPerLeg + i;
            visualization_msgs::Marker marker;
            marker.header.frame_id = odomFrame_;
            marker.header.stamp = timeStamp;
            marker.ns = ns;
            marker.id = id;
            marker.type = shape;
            marker.action = visualization_msgs::Marker::ADD;

            marker.pose.position.x = sampled(0, id);
            marker.pose.position.y = sampled(1, id);
            marker.pose.position.z = heightFunction(id);

            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;

            marker.scale.x = 0.03;
            marker.scale.y = 0.03;
            marker.scale.z = 0.03;

            marker.color.r = rgb[0];
            marker.color.g = rgb[1];
            marker.color.b = rgb[2];
            marker.color.a = 1.0;

            markerArray.markers.push_back(marker);
        }
    }

    markerPublisher_.publish(markerArray);
}

}  // namespace bobnet_visualization