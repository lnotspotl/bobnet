#pragma once

#include <bobnet_control/InverseKinematics.h>
#include <bobnet_control/anymal_c/AnymalCInfo.h>

namespace anymal_c {

using namespace bobnet_control;

class AnymalCInverseKinematics : public InverseKinematics {
   public:
    AnymalCInverseKinematics();

   private:
    vector_t solve_ik(matrix_t &footPositions) override;

    AnymalCInfo info_;
};

}  // namespace anymal_c