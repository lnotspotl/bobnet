#include <bobnet_controllers/CentralPatternGenerator.h>
#include <cmath>

namespace bobnet_controllers {

CentralPatternGenerator::CentralPatternGenerator(scalar_t period, scalar_t swingHeight, const vs &initial_offset)
    : period_(period), timeOffsets_(initial_offset), swingHeight_(swingHeight) {
    reset();
}

void CentralPatternGenerator::reset() { time_ = 0.0; }

void CentralPatternGenerator::step(const scalar_t dt) { time_ += dt; }

vs CentralPatternGenerator::computePhases() {
    // compute current leg time
    vs phases = timeOffsets_;
    for (size_t i = 0; i < phases.size(); ++i) {
        phases[i] += time_;
        phases[i] = std::fmod(phases[i] / period_, 1.0) * 2 * M_PI;
    }
    return phases;
}

vs CentralPatternGenerator::computePhases(vs &phase_offsets) {
    auto phases = computePhases();
    for (size_t i = 0; i < phases.size(); ++i) {
        phases[i] = std::fmod(phases[i] + phase_offsets[i], 2 * M_PI);
    }
    return phases;
}

vs CentralPatternGenerator::getObservation() {
    auto phases = computePhases();
    vs observation(8);
    for (int i = 0; i < 4; ++i) {
        observation[0 + i] = std::cos(phases[i]);
        observation[4 + i] = std::sin(phases[i]);
    }
    return observation;
}

vs CentralPatternGenerator::legHeights() {
    auto phases = computePhases();
    return computeLegHeights(phases);
}

vs CentralPatternGenerator::legHeights(vs &phase_offsets) {
    auto phases = computePhases(phase_offsets);
    return computeLegHeights(phases);
}

vs CentralPatternGenerator::computeLegHeights(vs &phases) {
    vs leg_heights(4);
    for (int i = 0; i < 4; ++i) {
        auto phase = phases[i];
        if (phase <= M_PI / 2) {  // swing - move up
            scalar_t time_up = phase * (2 / M_PI);
            leg_heights[i] = swingHeight_ * (-2 * std::pow(time_up, 3) + 3 * std::pow(time_up, 2));
        } else if (phase <= M_PI) {  // swing - move down
            scalar_t time_down = phase * (2 / M_PI) - 1;
            leg_heights[i] = swingHeight_ * (2 * std::pow(time_down, 3) - 3 * std::pow(time_down, 2) + 1.0);
        } else {  // stance
            leg_heights[i] = 0.0;
        }
    }

    return leg_heights;
}

}  // namespace bobnet_controllers
