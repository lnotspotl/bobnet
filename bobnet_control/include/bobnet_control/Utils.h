#pragma once

#include <string>
#include <torch/script.h>

namespace bobnet_control {

using torch::jit::script::Module;

Module loadTorchModel(const std::string& modelPath);

}  // namespace bobnet_controls