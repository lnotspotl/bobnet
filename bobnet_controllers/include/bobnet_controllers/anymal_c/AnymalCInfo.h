#pragma once

#include <string>
#include <bobnet_core/Types.h>

namespace anymal_c {

using namespace bobnet_core;

struct AnymalCInfo {
    /* Leg Geometry */
    scalar_t d2 = 0.19716;
    scalar_t a3 = 0.285;
    scalar_t a4 = 0.34923;

    /* Default foot positions expressed w.r.t hip frame*/
    vvs defaultPositions_ = {
        {0.00790786, 0.05720384, -0.573},
        {-0.00790786, 0.05720384, -0.573},
        {0.00790786, -0.05720384, -0.573},
        {-0.00790786, -0.05720384, -0.573},
    };

    /* Joint names */
    std::vector<std::string> jointNames{"LF_HAA", "LF_HFE", "LF_KFE", "LH_HAA", "LH_HFE", "LH_KFE",
                                        "RF_HAA", "RF_HFE", "RF_KFE", "RH_HAA", "RH_HFE", "RH_KFE"};

    /* Default joint angles for stand controller */
    vs standControllerJointAngles{0.0, 0.4, -0.8 /*LF*/, 0.0, -0.4, 0.8 /*LH*/,
                                  0.0, 0.4, -0.8 /*RF*/, 0.0, -0.4, 0.8 /*RH*/};

    /* Swing height */
    scalar_t swingHeight = 0.2;

    /* CPG cycle period */
    scalar_t cpgPeriod = 0.6;

    /* CPG phase offset - LF, LH, RF, RH*/
    vs cpgPhaseOffsets{0.0, cpgPeriod/2.0, cpgPeriod/2.0, 0.0};
};

}  // namespace anymal_c