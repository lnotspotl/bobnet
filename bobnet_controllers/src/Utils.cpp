#include <bobnet_controllers/Utils.h>

namespace bobnet_controllers {

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

}  // namespace bobnet_controllers
