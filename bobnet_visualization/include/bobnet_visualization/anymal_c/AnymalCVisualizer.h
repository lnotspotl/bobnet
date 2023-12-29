#pragma once

#include <bobnet_visualization/RvizVisualizer.h>

namespace bobnet_visualization {

class AnymalCVisualizer : public RvizVisualizer {
   public:
    AnymalCVisualizer();
    void visualize(const bobnet_core::State &state, const Eigen::MatrixXd &sampled,
                   const at::Tensor &nnPointsReconstructed);

   private:
    ros::Publisher markerPublisher_;

    void publishBaseTransform(ros::Time timeStamp, const bobnet_core::State &state);
    void publishJointAngles(ros::Time timeStamp, const bobnet_core::State &state);
    void publishMarkers(ros::Time timeStamp, const Eigen::MatrixXd &sampled, const at::Tensor &points,
                        const bobnet_core::State &state, size_t color_idx, const std::string &prefix, bool fromTensor);
};

}  // namespace bobnet_visualization