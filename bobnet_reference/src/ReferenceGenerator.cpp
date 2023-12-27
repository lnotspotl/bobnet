#include <bobnet_reference/ReferenceGenerator.h>

namespace bobnet_reference {

/*******************************************************************************/
/*******************************************************************************/
/*******************************************************************************/
void JoystickReferenceGenerator::callback(const sensor_msgs::Joy &msg) {
    referenceDesired_ = {msg.axes[xIndex_] * xScale_, msg.axes[yIndex_] * yScale_, msg.axes[yawIndex_] * yawScale_};
}

/*******************************************************************************/
/*******************************************************************************/
/*******************************************************************************/
VelocityCommand JoystickReferenceGenerator::getVelocityReference(scalart dt) {
    scalar_t x_diff = referenceDesired_.velocity_x - reference_.velocity_x;
    scalar_t y_diff = referenceDesired_.velocity_y - reference_.velocity_y;
    scalar_t yaw_diff = referenceDesired_.yaw_rate - reference_.yaw_rate;

    scalar_t x_step = std::sgn(x_diff) * std::min(std::abs(x_diff), rampedVelocity_ * dt);
    scalar_t y_step = std::sgn(y_diff) * std::min(std::abs(y_diff), rampedVelocity_ * dt);
    scalar_t yaw_step = std::sgn(yaw_diff) * std::min(std::abs(yaw_diff), rampedVelocity_ * dt);

    reference_.velocity_x += x_step;
    reference_.velocity_y += y_step;
    reference_.yaw_rate += yaw_step;

    return reference_;
}

/*******************************************************************************/
/*******************************************************************************/
/*******************************************************************************/
void TwistReferenceGenerator::callback(const sensor_msgs::Twist &msg) {
    reference_ = {msg.linear.x, msg.linear.y, msg.angular.z};
}

}  // namespace bobnet_reference