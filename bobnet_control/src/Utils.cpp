#include <bobnet_control/Utils.h>

#include <bobnet_control/anymal_c/AnymalCInverseKinematics.h>
#include <bobnet_config/utils.h>

#include <ros/ros.h>

namespace bobnet_control {

Module loadTorchModel(const std::string &modelPath) {
    torch::jit::script::Module model;
    try {
        model = torch::jit::load(modelPath);
    } catch (const c10::Error &e) {
        std::cerr << "Could not load model from: " << modelPath << std::endl;
        throw std::runtime_error("Could not load model");
    }
    return model;
}

std::unique_ptr<InverseKinematics> getInverseKinematics(const std::string &robotName) {
    if (robotName == "anymal_c") {
        return std::unique_ptr<InverseKinematics>(new anymal_c::AnymalCInverseKinematics());
    }

    throw std::runtime_error("Unknown robot name:  " + robotName);
}

std::unique_ptr<CentralPatternGenerator> getCentralPatternGenerator() {
    auto period = bobnet_config::fromRosConfigFile<scalar_t>("bob_controller/cpg/period");
    auto time_offset = bobnet_config::fromRosConfigFile<vector_t>("bob_controller/cpg/time_offsets");
    auto swingHeight = bobnet_config::fromRosConfigFile<scalar_t>("bob_controller/cpg/swing_height");

    return std::unique_ptr<CentralPatternGenerator>(new CentralPatternGenerator(period, swingHeight, time_offset));
}

std::unique_ptr<bobnet_reference::ReferenceGenerator> getReferenceGenerator() {
    ros::NodeHandle nh;
    std::string topic = "joy";
    scalar_t rampedVelocity = 0.3;

    return std::unique_ptr<bobnet_reference::ReferenceGenerator>(
        new bobnet_reference::JoystickReferenceGenerator(nh, topic, rampedVelocity));
}

std::unique_ptr<bobnet_gridmap::GridmapInterface> getGridmapInterface() {
    ros::NodeHandle nh;
    auto topic = bobnet_config::fromRosConfigFile<std::string>("gridmap_topic");

    return std::unique_ptr<bobnet_gridmap::GridmapInterface>(new bobnet_gridmap::GridmapInterface(nh, topic));
}

}  // namespace bobnet_control
