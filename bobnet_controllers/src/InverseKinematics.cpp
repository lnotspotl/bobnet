#include <bobnet_controllers/InverseKinematics.h>

namespace bobnet_controllers {

vs InverseKinematics::solve(vs &legHeightDiffs) {
    vvs positions = defaultPositions_;
    for (size_t i = 0; i < positions.size(); ++i) {
        positions[i][2] += legHeightDiffs[i];
    }
    return solve_ik(positions);
}

}  // namespace bobnet_controllers