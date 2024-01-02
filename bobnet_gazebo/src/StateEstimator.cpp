#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/parsers/urdf.hpp"

#include <bobnet_gazebo/StateEstimator.h>

#include <functional>
#include <string>

namespace gazebo {

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
void StateEstimator::Load(physics::ModelPtr robot, sdf::ElementPtr sdf) {
    // setup Gazebo event callback
    updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&StateEstimator::OnUpdate, this));

    // setup ROS publisher
    ros::NodeHandle nh;
    std::string stateTopic = robot->GetName() + "/state";
    if (!nh.getParam("/bobnet_gazebo/state_topic", stateTopic)) {
        ROS_WARN_STREAM("No state topic specified for " << robot->GetName() << ", using default: " << stateTopic);
    }
    statePublisher_ = nh.advertise<bobnet_msgs::RobotState>(stateTopic, 2);

    // get baselink name
    std::string baseLinkName = "base_link";
    if (!nh.getParam("/bobnet_gazebo/base_link", baseLinkName)) {
        ROS_WARN_STREAM("No base link specified for " << robot->GetName() << ", using default: " << baseLinkName);
    }

    // get baselink ptr
    baseLinkPtr_ = robot->GetChildLink(baseLinkName);

    // get update rate
    updateRate_ = 100.0;
    if (!nh.getParam("/bobnet_gazebo/state_update_rate", updateRate_)) {
        ROS_WARN_STREAM("No update rate specified for " << robot->GetName() << ", using default: " << updateRate_);
    }

    // Load foot names
    auto loadFootName = [&nh](std::string &footName, const std::string &defaultName, const std::string &footNameParam) {
        if (!nh.getParam(footNameParam, footName)) {
            ROS_WARN_STREAM("No footname specified for " << defaultName << ", using default: " << defaultName);
            footName = defaultName;
        }
    };
    loadFootName(lf_name_, "LF_FOOT", "/bobnet_gazebo/lf_foot");
    loadFootName(lh_name_, "LH_FOOT", "/bobnet_gazebo/lh_foot");
    loadFootName(rf_name_, "RF_FOOT", "/bobnet_gazebo/rf_foot");
    loadFootName(rh_name_, "RH_FOOT", "/bobnet_gazebo/rh_foot");

    // prepare pinocchio model
    std::string urdfString;
    nh.getParam("/robot_description", urdfString);
    ROS_INFO_STREAM("Loading URDF for " << robot->GetName());
    pinocchio::urdf::buildModelFromXML(urdfString, pinocchio::JointModelFreeFlyer(), model_);
    ROS_INFO_STREAM("Loaded URDF for " << robot->GetName());
    data_ = pinocchio::Data(model_);

    // load joint pointers
    joints_.clear();
    for (size_t i = 2; i < model_.names.size(); ++i) {
        auto &jointName = model_.names[i];
        joints_.push_back(robot->GetJoint(jointName));
    }

    lastJointAngles_.clear();
    robot_ = robot;
    lastSimTime_ = robot_->GetWorld()->SimTime();
}

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
void StateEstimator::OnUpdate() {
    common::Time currentSimTime = robot_->GetWorld()->SimTime();
    double dt = (currentSimTime - lastSimTime_).Double();

    if (dt < 1.0 / updateRate_) {
        return;
    }

    bobnet_msgs::RobotState stateMsg;
    fillStateMsg(stateMsg, dt);
    statePublisher_.publish(stateMsg);

    lastSimTime_ = currentSimTime;
}

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
void StateEstimator::fillStateMsg(bobnet_msgs::RobotState &msg, double dt) {
    auto &msg_ref = msg;

    // base link world position
    auto &baseLinkPose = baseLinkPtr_->WorldPose();
    auto posBase = baseLinkPose.Pos();
    msg_ref.base_position_world = {posBase.X(), posBase.Y(), posBase.Z()};

    // base link world orientation
    auto quatBase = baseLinkPose.Rot();
    msg_ref.base_orientation_world = {quatBase.X(), quatBase.Y(), quatBase.Z(), quatBase.W()};
    auto baseOrientationMat =
        Eigen::Quaterniond(quatBase.W(), quatBase.X(), quatBase.Y(), quatBase.Z()).toRotationMatrix();
    if (firstUpdate_) {
        lastBaseOrientationMat_ = baseOrientationMat;
        firstUpdate_ = false;
    }

    // Get base linear velocity expressed in the base frame
    const auto lin_vel_world = baseLinkPtr_->WorldLinearVel();
    Eigen::Vector3d lin_vel_base = {lin_vel_world.X(), lin_vel_world.Y(), lin_vel_world.Z()};
    lin_vel_base = baseOrientationMat.transpose() * lin_vel_base;
    // const auto lin_vel_base = quatBase.RotateVectorReverse(lin_vel_world);
    msg_ref.base_lin_vel_b = {lin_vel_base[0], lin_vel_base[1], lin_vel_base[2]};

    // compute angular velocity
    Eigen::Vector3d ang_vel = mat2aa(baseOrientationMat * lastBaseOrientationMat_.transpose()) / dt;

    // Express angular velocity in the base frame
    const auto ang_vel_base = baseOrientationMat.transpose() * ang_vel;
    msg_ref.base_ang_vel_b = {ang_vel_base[0], ang_vel_base[1], ang_vel_base[2]};

    // Get normalized gravity vecctor expressed in the base frame
    Eigen::Vector3d normalized_gravity = {0, 0, -1.0};
    normalized_gravity = baseOrientationMat.transpose() * normalized_gravity;
    msg_ref.normalized_gravity_b = {normalized_gravity[0], normalized_gravity[1], normalized_gravity[2]};

    // Get joint positions
    for (size_t i = 0; i < joints_.size(); ++i) {
        msg_ref.joint_pos[i] = joints_[i]->Position(0);
    }
    if (lastJointAngles_.size() != joints_.size()) {
        lastJointAngles_.resize(joints_.size());
        for (size_t i = 0; i < joints_.size(); ++i) {
            lastJointAngles_[i] = msg_ref.joint_pos[i];
        }
    }

    // Get joint velocities
    for (int i = 0; i < joints_.size(); ++i) {
        double current_angle = msg_ref.joint_pos[i];
        double last_angle = lastJointAngles_[i];
        double velocity = (current_angle - last_angle) / dt;
        msg_ref.joint_vel[i] = velocity;
        lastJointAngles_[i] = current_angle;
    }

    // compute forward kinematics
    Eigen::VectorXd q(model_.nq);
    q[0] = posBase.X();
    q[1] = posBase.Y();
    q[2] = posBase.Z();
    q[3] = quatBase.X();
    q[4] = quatBase.Y();
    q[5] = quatBase.Z();
    q[6] = quatBase.W();
    for (int i = 0; i < 12; ++i) {
        q[7 + i] = msg_ref.joint_pos[i];
    }
    pinocchio::framesForwardKinematics(model_, data_, q);

    // LF foot position
    const auto &lf_foot_pos = data_.oMf[model_.getFrameId(lf_name_)].translation();
    msg_ref.lf_position_world = {lf_foot_pos[0], lf_foot_pos[1], lf_foot_pos[2]};

    const auto &lh_foot_pos = data_.oMf[model_.getFrameId(lh_name_)].translation();
    msg_ref.lh_position_world = {lh_foot_pos[0], lh_foot_pos[1], lh_foot_pos[2]};

    const auto &rf_foot_pos = data_.oMf[model_.getFrameId(rf_name_)].translation();
    msg_ref.rf_position_world = {rf_foot_pos[0], rf_foot_pos[1], rf_foot_pos[2]};

    const auto &rh_foot_pos = data_.oMf[model_.getFrameId(rh_name_)].translation();
    msg_ref.rh_position_world = {rh_foot_pos[0], rh_foot_pos[1], rh_foot_pos[2]};

    // update publish time
    lastBaseOrientationMat_ = baseOrientationMat;
}

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
Eigen::Vector3d StateEstimator::mat2aa(const Eigen::Matrix3d &R) {
    Eigen::AngleAxisd aa(R);
    return aa.axis() * aa.angle();
}

GZ_REGISTER_MODEL_PLUGIN(StateEstimator);
}  // namespace gazebo