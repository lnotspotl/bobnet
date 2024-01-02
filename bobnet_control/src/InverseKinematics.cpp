#include <bobnet_control/InverseKinematics.h>

namespace bobnet_control {

vector_t InverseKinematics::solve(vector_t &legHeightDiffs) {
    matrix_t positions = defaultPositions_;
    for (size_t i = 0; i < 4; ++i) {
        positions(i,2) += legHeightDiffs[i];
    }

    matrix_t positions_mat = matrix_t::Zero(4, 3);
    for (size_t i = 0; i < 4; ++i) {
        positions_mat.row(i) = vector3_t(positions(i,0), positions(i,1), positions(i,2));
    }

    auto out = solve_ik(positions_mat);

    vector_t out_angles(12);
    for (size_t i = 0; i < 12; ++i) {
        out_angles[i] = out[i];
    }

    return out_angles;
}

}  // namespace bobnet_control