#include <bobnet_control/InverseKinematics.h>

#include <bobnet_config/utils.h>

namespace bobnet_control {

vector_t InverseKinematics::solve(vector_t &legHeightDiffs) {
    matrix_t positions = defaultStance_;
    positions.col(2) += legHeightDiffs;
    auto angles = solve_ik(positions);
    return angles;
}

AnymalCInverseKinematics::AnymalCInverseKinematics() {
    d2_ = bobnet_config::fromRosConfigFile<scalar_t>("bob_controller/ik/d2");
    a3_ = bobnet_config::fromRosConfigFile<scalar_t>("bob_controller/ik/a3");
    a4_ = bobnet_config::fromRosConfigFile<scalar_t>("bob_controller/ik/a4");

    defaultStance_ = bobnet_config::fromRosConfigFile<matrix_t>("bob_controller/default_stance");
}

vector_t AnymalCInverseKinematics::solve_ik(matrix_t &footPositions) {
    vector_t joint_angles(12);
    for (int i = 0; i < 4; ++i) {
        const scalar_t d2 = i < 2 ? d2_ : -d2_;
        const scalar_t a3 = a3_;
        const scalar_t a4 = a4_;

        const scalar_t x = footPositions(i, 0);
        const scalar_t y = footPositions(i, 1);
        const scalar_t z = footPositions(i, 2);

        const scalar_t E = y * y + z * z - d2 * d2;
        const scalar_t E_sqrt = sqrt(E);
        scalar_t theta1 = atan2(E_sqrt, d2) + atan2(z, y);

        scalar_t D = (E + x * x - a3 * a3 - a4 * a4) / (2.0 * a3 * a4);
        D = std::max(static_cast<scalar_t>(-1.0), std::min(static_cast<scalar_t>(1.0), D));
        scalar_t theta4 = -atan2(sqrt(1.0 - D * D), D);
        constexpr scalar_t theta4_offset = 0.254601;
        scalar_t theta4_final = theta4 + theta4_offset;

        if (i % 2 == 1) {  // hind legs
            theta4 *= -1.0;
            theta4_final *= -1.0;
        }

        scalar_t theta3 = atan2(-x, E_sqrt) - atan2(a4 * sin(theta4), a3 + a4 * cos(theta4));

        joint_angles.segment<3>(3 * i) = (vector3_t() << theta1, theta3, theta4_final).finished();
    }

    return joint_angles;
}

SpotInverseKinematics::SpotInverseKinematics() {
    d2_ = bobnet_config::fromRosConfigFile<scalar_t>("bob_controller/ik/d2");
    a3_ = bobnet_config::fromRosConfigFile<scalar_t>("bob_controller/ik/a3");
    a4_ = bobnet_config::fromRosConfigFile<scalar_t>("bob_controller/ik/a4");

    defaultStance_ = bobnet_config::fromRosConfigFile<matrix_t>("bob_controller/default_stance");
}

vector_t SpotInverseKinematics::solve_ik(matrix_t &footPositions) {
    vector_t joint_angles(12);
    for (int i = 0; i < 4; ++i) {
        const scalar_t d2 = i < 2 ? d2_ : -d2_;
        const scalar_t a3 = a3_;
        const scalar_t a4 = a4_;

        const scalar_t x = footPositions(i, 0);
        const scalar_t y = footPositions(i, 1);
        const scalar_t z = footPositions(i, 2);

        const scalar_t E = y * y + z * z - d2 * d2;
        const scalar_t E_sqrt = sqrt(E);
        scalar_t theta1 = atan2(E_sqrt, d2) + atan2(z, y);

        scalar_t D = (E + x * x - a3 * a3 - a4 * a4) / (2.0 * a3 * a4);
        D = std::max(static_cast<scalar_t>(-1.0), std::min(static_cast<scalar_t>(1.0), D));
        scalar_t theta4 = -atan2(sqrt(1.0 - D * D), D);
        constexpr scalar_t theta4_offset = -0.0779666;
        scalar_t theta4_final = theta4 + theta4_offset;

        scalar_t theta3 = atan2(-x, E_sqrt) - atan2(a4 * sin(theta4), a3 + a4 * cos(theta4));
        constexpr scalar_t theta3_offset = 0.0779666;

        scalar_t theta3_final = theta3 + theta3_offset;

        joint_angles.segment<3>(3 * i) = (vector3_t() << theta1, theta3_final, theta4_final).finished();
    }

    return joint_angles;
}

std::unique_ptr<InverseKinematics> getInverseKinematicsUnique() {
    auto robotName = bobnet_config::fromRosConfigFile<std::string>("robot_name");

    if (robotName == "anymal_c") {
        return std::unique_ptr<InverseKinematics>(new AnymalCInverseKinematics());
    }

    if (robotName == "spot") {
        return std::unique_ptr<InverseKinematics>(new SpotInverseKinematics());
    }

    throw std::runtime_error("Unknown robot name:  " + robotName);
}

std::shared_ptr<InverseKinematics> getInverseKinematicsShared() {
    return std::shared_ptr<InverseKinematics>(getInverseKinematicsUnique().release());
}

}  // namespace bobnet_control