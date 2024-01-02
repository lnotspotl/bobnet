#pragma once

// STL
#include <vector>
#include <string>

// ROS
#include <ros/ros.h>
#include <bobnet_msgs/RobotState.h>

// Gazebo plugin
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

// Pinocchio
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/parsers/urdf.hpp"

// Eigen
#include <Eigen/Dense>

namespace gazebo {

using scalar_t = float;
using Vector3 = Eigen::Matrix<scalar_t, 3, 1>;
using Matrix3 = Eigen::Matrix<scalar_t, 3, 3>;
using VectorX = Eigen::Matrix<scalar_t, Eigen::Dynamic, 1>;
using AngleAxis = Eigen::AngleAxis<scalar_t>;
using Quaternion = Eigen::Quaternion<scalar_t>;

class StateEstimator : public ModelPlugin {
   public:
    void Load(physics::ModelPtr _parent, sdf::ElementPtr sdf);
    void OnUpdate();

   private:
    void fillStateMsg(bobnet_msgs::RobotState &msg, scalar_t dt);

    inline scalar_t cast_dt(const common::Time &t_diff) {
        return static_cast<scalar_t>(t_diff.sec) + static_cast<scalar_t>(t_diff.nsec) * 1e-9;
    }

    /* Convert a rotation matrix to its angle-axis representation */
    Vector3 mat2aa(const Matrix3 &R);

    // callback function for Gazebo
    event::ConnectionPtr updateConnection;

    // robot state publisher
    ros::Publisher statePublisher_;

    physics::ModelPtr robot_;

    // robot base link ptr
    physics::LinkPtr baseLinkPtr_;

    // joint pointers
    std::vector<physics::JointPtr> joints_;
    std::vector<scalar_t> lastJointAngles_;

    // pinocchio model
    pinocchio::ModelTpl<scalar_t> model_;
    pinocchio::DataTpl<scalar_t> data_;

    std::string lf_name_;
    std::string rf_name_;
    std::string lh_name_;
    std::string rh_name_;

    // update rate
    scalar_t updateRate_;

    bool firstUpdate_ = true;

    // last yaw angle
    Matrix3 lastBaseOrientationMat_;
    common::Time lastSimTime_;
};

}  // namespace gazebo