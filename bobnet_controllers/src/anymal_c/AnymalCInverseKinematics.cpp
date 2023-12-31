#include <bobnet_controllers/anymal_c/AnymalCInverseKinematics.h>

#include <cmath>

namespace anymal_c {

AnymalCInverseKinematics::AnymalCInverseKinematics() { defaultPositions_ = info_.defaultPositions_; }

vector_t AnymalCInverseKinematics::solve_ik(matrix_t &footPositions) {

    vector_t joint_angles(12);
    for (int i = 0; i < 4; ++i) {
        scalar_t d2 = i < 2 ? info_.d2 : -info_.d2;
        scalar_t a3 = info_.a3;
        scalar_t a4 = info_.a4;

        const scalar_t x = footPositions(i, 0);  
        const scalar_t y = footPositions(i, 1);  
        const scalar_t z = footPositions(i, 2);

        const scalar_t E = y * y + z * z - d2 * d2;
        const scalar_t E_sqrt = sqrt(E);
        scalar_t theta1 = atan2(E_sqrt, d2) + atan2(z, y);

        double D = (E + x * x - a3 * a3 - a4 * a4) / (2.0 * a3 * a4);
        D = std::max(-1.0, std::min(1.0, D));
        double theta4 = -atan2(sqrt(1.0 - D * D), D);
        constexpr scalar_t theta4_offset = 0.254601;
        double theta4_final = theta4 + theta4_offset;

        if (i % 2 == 1) {  // hind legs
            theta4 *= -1.0;
            theta4_final *= -1.0;
        }

        double theta3 = atan2(-x, E_sqrt) - atan2(a4 * sin(theta4), a3 + a4 * cos(theta4));

        joint_angles[3 * i + 0] = theta1;
        joint_angles[3 * i + 1] = theta3;
        joint_angles[3 * i + 2] = theta4_final;

        joint_angles.segment<3>(3 * i) = (vector3_t() << theta1, theta3, theta4_final).finished();
    }

    return joint_angles;
}

}  // namespace anymal_c