#pragma once

#include <bobnet_controllers/Types.h>

namespace bobnet_controllers {

class InverseKinematics {
   public:
    /* Solve inverse kinematics */
    vs solve(vs &legHeightDiffs);

   protected:
    /* Default foot positons expressed w.r.t hip frame */
    vvs defaultPositions_;

   private:
    /* Solve inverse kinematics for given foot positions expressed w.r.t. hip frame*/
    virtual vs solve_ik(vvs &footPositions) = 0;
};

}  // namespace bobnet_controllers