#pragma once

#include <string>
#include <bobnet_core/Types.h>

namespace bobnet_control {

using namespace bobnet_core;

// enum class ControllerType { STAND, SIT };

class Controller {

    public:
        virtual ~Controller() = default;

        virtual void sendCommand(const scalar_t dt) = 0;

        virtual void visualize() = 0;

        virtual void changeController(const std::string &controllerType) = 0;

        virtual bool isSupported(const std::string &controllerType) = 0;
};

} // namespace bobnet_control