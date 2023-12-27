#pragma once

#include <bobnet_controllers/InverseKinematics.h>
#include <bobnet_controllers/anymal_c/AnymalCInfo.h>

namespace anymal_c {

using namespace bobnet_controllers;

class AnymalCInverseKinematics : public InverseKinematics {
   public:
    AnymalCInverseKinematics();

   private:
    vs solve_ik(vvs &footPositions) override;

    AnymalCInfo info_;
};

}  // namespace anymal_c