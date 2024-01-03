#include <bobnet_visualization/anymal_c/AnymalCVisualizer.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace bobnet_visualization {

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
AnymalCVisualizer::AnymalCVisualizer() : RvizVisualizer() {
    markerPublisher_ = ros::NodeHandle().advertise<visualization_msgs::MarkerArray>("visualization_marker", 1);
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void AnymalCVisualizer::visualize(const bobnet_core::State &state, const Eigen::MatrixXd &sampled,
                                  const at::Tensor &nnPointsReconstructed) {
    ros::Time timeStamp = ros::Time::now();
    publishBaseTransform(timeStamp, state);
    publishJointAngles(timeStamp, state);

    // publishMarkers(timeStamp, sampled, 0, "ground_truth",
    //                [&](size_t id) { return -(sampled(2, id) / 1.0 + 0.5 - state.basePositionWorld[2]); });
    // publishMarkers(timeStamp, sampled, 1, "nn_reconstructed", [&](size_t id) {
    //     float out = -(nnPointsReconstructed[id].item<float>() / 1.0 + 0.5 - state.basePositionWorld[2]);
    //     return static_cast<scalar_t>(out);
    // });
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void AnymalCVisualizer::publishBaseTransform(ros::Time timeStamp, const bobnet_core::State &state) {
    geometry_msgs::TransformStamped baseToWorldTransform;
    baseToWorldTransform.header.stamp = timeStamp;
    baseToWorldTransform.header.frame_id = worldFrame_;
    baseToWorldTransform.child_frame_id = "base";

    baseToWorldTransform.transform.translation.x = state.basePositionWorld[0];
    baseToWorldTransform.transform.translation.y = state.basePositionWorld[1];
    baseToWorldTransform.transform.translation.z = state.basePositionWorld[2];

    baseToWorldTransform.transform.rotation.x = state.baseOrientationWorld[0];
    baseToWorldTransform.transform.rotation.y = state.baseOrientationWorld[1];
    baseToWorldTransform.transform.rotation.z = state.baseOrientationWorld[2];
    baseToWorldTransform.transform.rotation.w = state.baseOrientationWorld[3];

    tfBroadcaster_.sendTransform(baseToWorldTransform);
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void AnymalCVisualizer::publishJointAngles(ros::Time timeStamp, const bobnet_core::State &state) {
    auto &jointAngles = state.jointPositions;
    std::map<std::string, double> jointPositions{
        {"LF_HAA", jointAngles[0]}, {"LF_HFE", jointAngles[1]},  {"LF_KFE", jointAngles[2]},
        {"LH_HAA", jointAngles[3]}, {"LH_HFE", jointAngles[4]},  {"LH_KFE", jointAngles[5]},
        {"RF_HAA", jointAngles[6]}, {"RF_HFE", jointAngles[7]},  {"RF_KFE", jointAngles[8]},
        {"RH_HAA", jointAngles[9]}, {"RH_HFE", jointAngles[10]}, {"RH_KFE", jointAngles[11]}};

    robotStatePublisherPtr_->publishTransforms(jointPositions, timeStamp);
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void AnymalCVisualizer::publishMarkers(ros::Time timeStamp, const Eigen::MatrixXd &sampled, size_t color_idx,
                                       const std::string &prefix, std::function<scalar_t(size_t)> f) {
    std::vector<std::array<double, 4>> colors = {
        {1.0, 0.0, 0.0, 1.0}, {0.0, 1.0, 0.0, 1.0}, {0.0, 0.0, 1.0, 1.0}, {1.0, 1.0, 0.0, 1.0}};

    uint32_t shape = visualization_msgs::Marker::SPHERE;
    visualization_msgs::MarkerArray markerArray;

    for (size_t leg_idx = 0; leg_idx < 4; ++leg_idx) {
        const std::string ns = prefix + "_sampled_points_" + std::to_string(leg_idx);
        for (size_t i = 0; i < 52; ++i) {
            size_t id = leg_idx * 52 + i;
            visualization_msgs::Marker marker;
            marker.header.frame_id = "odom";
            marker.header.stamp = timeStamp;
            marker.ns = ns;
            marker.id = id;
            marker.type = shape;
            marker.action = visualization_msgs::Marker::ADD;

            marker.pose.position.x = sampled(0, id);
            marker.pose.position.y = sampled(1, id);
            marker.pose.position.z = f(id);

            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;

            marker.scale.x = 0.03;
            marker.scale.y = 0.03;
            marker.scale.z = 0.03;

            marker.color.r = colors[color_idx][0];
            marker.color.g = colors[color_idx][1];
            marker.color.b = colors[color_idx][2];
            marker.color.a = colors[color_idx][3];

            markerArray.markers.push_back(marker);
        }
    }

    markerPublisher_.publish(markerArray);
}

}  // namespace bobnet_visualization