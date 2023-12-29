#pragma once

#include <bobnet_core/Types.h>

namespace bobnet_controllers {

using namespace bobnet_core;

class CentralPatternGenerator {
   public:
    CentralPatternGenerator(scalar_t period, scalar_t swingHeight, const vs &initial_offset);

    void reset();
    void step(const scalar_t dt);
    vs computePhases();
    vs computePhases(vs &phase_offsets);
    vs getObservation();

    vs legHeights();
    vs legHeights(vs &phase_offsets);

   private:
    vs computeLegHeights(vs &phases);

    scalar_t time_;
    scalar_t period_;
    scalar_t swingHeight_;
    vs timeOffsets_;
};

}  // namespace bobnet_controllers