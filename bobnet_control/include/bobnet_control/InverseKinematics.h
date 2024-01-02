#pragma once

#include <bobnet_core/Types.h>

namespace bobnet_control {

using namespace bobnet_core;

class InverseKinematics {
   public:
    /* Solve inverse kinematics */
    vector_t solve(vector_t &legHeightDiffs);

   protected:
    /* Default foot positons expressed w.r.t hip frame */
    matrix_t defaultPositions_;

   private:
    /* Solve inverse kinematics for given foot positions expressed w.r.t. hip frame*/
    virtual vector_t solve_ik(matrix_t &footPositions) = 0;
};

}  // namespace bobnet_control