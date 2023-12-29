#pragma once

#include <string>
#include <torch/script.h>

namespace bobnet_controllers {

using torch::jit::script::Module;

Module loadTorchModel(const std::string& modelPath);

}  // namespace bobnet_controllerss