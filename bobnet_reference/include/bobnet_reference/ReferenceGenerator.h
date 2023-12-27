#pragma once

#include <string>

#include <ros/ros.h>
#include <bobnet_controllers/Types.h>

// Ros messages
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Twist.h>

namespace bobnet_reference {

using namespace bobnet_controllers;

struct VelocityCommand {
    scalar_t velocity_x = 0.0;
    scalar_t velocity_y = 0.0;
    scalar_t yaw_rate = 0.0;
};

class ReferenceGenerator {
   public:
    virtual VelocityCommand getVelocityReference(scalar_t dt) = 0;
};

template <typename T>
class RosReferenceGenerator : public ReferenceGenerator {
   public:
    RosReferenceGenerator(ros::NodeHandle &nh, const std::string &topic) {
        subscriber_ = nh.subscribe(topic, 1, &RosReferenceGenerator::callback, this);
    }

    VelocityCommand getVelocityReference(scalar_t dt) override { return reference_; }

   private:
    virtual void callback(const T &msg) = 0;

    ros::Subscriber subscriber_;
    VelocityCommand reference_;
};

class JoystickReferenceGenerator : public RosReferenceGenerator<sensor_msgs::Joy> {
   public:
    JoystickReferenceGenerator(ros::NodeHandle &nh, const std::string &topic, scalar_t rampedVelocity,
                               size_t xIndex = 0, size_t yIndex = 1, size_t yawIndex = 2, scalar_t xScale = 1.0,
                               scalar_t yScale = 1.0, scalar_t yawScale = 1.0)
        : RosReferenceGenerator<sensor_msgs::Joy>(nh, topic),
          rampedVelocity_(rampedVelocity),
          xIndex_(xIndex),
          yIndex_(yIndex),
          yawIndex_(yawIndex),
          xScale_(xScale),
          yScale_(yScale),
          yawScale_(yawScale) {}

    VelocityCommand getVelocityReference(scalar_t dt) override;

   private:
    scalar_t rampedVelocity_;
    VelocityCommand referenceDesired_;

    size_t xIndex_;
    size_t yIndex_;
    size_t yawIndex_;

    scalar_t xScale_;
    scalar_t yScale_;
    scalar_t yawScale_;

    void callback(const sensor_msgs::Joy &msg) override;
};

class TwistReferenceGenerator : public RosReferenceGenerator<sensor_msgs::Twist> {
   public:
    TwistReferenceGenerator(ros::NodeHandle &nh, const std::string &topic)
        : RosReferenceGenerator<sensor_msgs::Twist>(nh, topic) {}

   private:
    void callback(const sensor_msgs::Twist &msg) override;
};

}  // namespace bobnet_reference