#pragma once

#include <bobnet_visualization/RvizVisualizer.h>

#include <bobnet_core/Types.h>
#include <functional>

namespace bobnet_visualization {

using namespace bobnet_core;

class AnymalCVisualizer : public RvizVisualizer {
   public:
    AnymalCVisualizer();
    void visualize(const bobnet_core::State &state, const Eigen::MatrixXd &sampled,
                   const at::Tensor &nnPointsReconstructed);

   private:
    ros::Publisher markerPublisher_;

    void publishBaseTransform(ros::Time timeStamp, const bobnet_core::State &state);
    void publishJointAngles(ros::Time timeStamp, const bobnet_core::State &state);
    void publishMarkers(ros::Time timeStamp, const Eigen::MatrixXd &sampled, size_t color_idx,
                        const std::string &prefix, std::function<scalar_t(size_t)> f);
};

}  // namespace bobnet_visualization