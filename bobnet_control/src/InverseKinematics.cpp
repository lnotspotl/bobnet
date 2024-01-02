#include <bobnet_control/InverseKinematics.h>

namespace bobnet_control {

vector_t InverseKinematics::solve(vector_t &legHeightDiffs) {
    matrix_t positions = defaultPositions_;

    // Update z positions
    positions.col(2) += legHeightDiffs;

    // Solve IK
    auto angles = solve_ik(positions);

    return angles;
}

}  // namespace bobnet_control