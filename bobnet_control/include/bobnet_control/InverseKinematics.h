#pragma once

#include <bobnet_core/Types.h>

#include <memory>

namespace bobnet_control {

using namespace bobnet_core;

class InverseKinematics {
   public:
    /* Solve inverse kinematics */
    vector_t solve(vector_t &legHeightDiffs);

   protected:
    /* Default foot positons expressed w.r.t hip frame */
    matrix_t defaultStance_;

   private:
    /* Solve inverse kinematics for given foot positions expressed w.r.t. hip frame*/
    virtual vector_t solve_ik(matrix_t &footPositions) = 0;
};

class AnymalCInverseKinematics : public InverseKinematics {
   public:
    AnymalCInverseKinematics();
    vector_t solve_ik(matrix_t &footPositions) override;

   private:
    scalar_t d2_;
    scalar_t a3_;
    scalar_t a4_;
};

std::unique_ptr<InverseKinematics> getInverseKinematicsUnique();

std::shared_ptr<InverseKinematics> getInverseKinematicsShared();

}  // namespace bobnet_control