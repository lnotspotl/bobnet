#pragma once

#include <string>
#include <torch/script.h>
#include <bobnet_control/InverseKinematics.h>
#include <bobnet_control/CentralPatternGenerator.h>
#include <bobnet_reference/ReferenceGenerator.h>
#include <bobnet_gridmap/GridmapInterface.h>
#include <memory>

namespace bobnet_control {

using torch::jit::script::Module;

Module loadTorchModel(const std::string& modelPath);

std::unique_ptr<InverseKinematics> getInverseKinematics(const std::string& robotName);
std::unique_ptr<CentralPatternGenerator> getCentralPatternGenerator();
std::unique_ptr<bobnet_reference::ReferenceGenerator> getReferenceGenerator();
std::unique_ptr<bobnet_gridmap::GridmapInterface> getGridmapInterface();

}  // namespace bobnet_controls