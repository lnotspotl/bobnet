#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include <bobnet_config/utils.h>

#include <bobnet_gazebo/StateEstimator.h>

#include <functional>
#include <string>

namespace gazebo {

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void StateEstimator::Load(physics::ModelPtr robot, sdf::ElementPtr sdf) {
    // setup Gazebo event callback
    updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&StateEstimator::OnUpdate, this));

    // setup ROS publisher
    ros::NodeHandle nh;
    auto stateTopic = bobnet_config::fromRosConfigFile<std::string>("state_topic");
    ROS_INFO_STREAM("Publishing state to " << stateTopic);
    statePublisher_ = nh.advertise<bobnet_msgs::RobotState>(stateTopic, 2);

    // get baselink name
    auto baseName = bobnet_config::fromRosConfigFile<std::string>("base_name");

    ROS_INFO_STREAM("Base name: " << baseName);

    // get baselink ptr
    baseLinkPtr_ = robot->GetChildLink(baseName);

    ROS_INFO_STREAM("Reading update rate");
    // get update rate
    updateRate_ = bobnet_config::fromRosConfigFile<scalar_t>("state_estimator/update_rate");

    auto footNames = bobnet_config::fromRosConfigFile<std::vector<std::string>>("foot_names");
    lf_name_ = footNames[0];
    lh_name_ = footNames[1];
    rf_name_ = footNames[2];
    rh_name_ = footNames[3];

    // prepare pinocchio model
    std::string urdfString;
    nh.getParam("/robot_description", urdfString);
    ROS_INFO_STREAM("Loading URDF for " << robot->GetName());

    // Load Model 
    pinocchio::Model m;
    pinocchio::urdf::buildModelFromXML(urdfString, pinocchio::JointModelFreeFlyer(), m);
    model_ = m.cast<scalar_t>();
    ROS_INFO_STREAM("Loaded URDF for " << robot->GetName());
    data_ = pinocchio::DataTpl<scalar_t>(model_);

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

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void StateEstimator::OnUpdate() {
    common::Time currentSimTime = robot_->GetWorld()->SimTime();
    scalar_t dt = cast_dt(currentSimTime - lastSimTime_);

    if (dt < static_cast<scalar_t>(1.0) / updateRate_) {
        return;
    }

    bobnet_msgs::RobotState stateMsg;
    fillStateMsg(stateMsg, dt);
    statePublisher_.publish(stateMsg);

    lastSimTime_ = currentSimTime;
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void StateEstimator::fillStateMsg(bobnet_msgs::RobotState &msg, scalar_t dt) {
    auto &msg_ref = msg;

    // base link world position
    auto &baseLinkPose = baseLinkPtr_->WorldPose();
    auto posBase = baseLinkPose.Pos();
    msg_ref.base_position_world = {posBase.X(), posBase.Y(), posBase.Z()};

    // base link world orientation
    auto quatBase = baseLinkPose.Rot();
    msg_ref.base_orientation_world = {quatBase.X(), quatBase.Y(), quatBase.Z(), quatBase.W()};
    auto baseOrientationMat = Quaternion(quatBase.W(), quatBase.X(), quatBase.Y(), quatBase.Z()).toRotationMatrix();
    if (firstUpdate_) {
        lastBaseOrientationMat_ = baseOrientationMat;
        firstUpdate_ = false;
    }

    // Get base linear velocity expressed in the base frame
    const auto lin_vel_world = baseLinkPtr_->WorldLinearVel();
    const auto lin_vel_base = quatBase.RotateVectorReverse(lin_vel_world);
    msg_ref.base_lin_vel_b = {lin_vel_base.X(), lin_vel_base.Y(), lin_vel_base.Z()};

    // compute angular velocity
    Vector3 ang_vel = mat2aa(baseOrientationMat * lastBaseOrientationMat_.transpose()) / dt;

    // Express angular velocity in the base frame
    const auto ang_vel_base = baseOrientationMat.transpose() * ang_vel;
    msg_ref.base_ang_vel_b = {ang_vel_base[0], ang_vel_base[1], ang_vel_base[2]};

    // Get normalized gravity vecctor expressed in the base frame
    const ignition::math::Vector3d normalized_gravity_world = {0.0, 0.0, -1.0};
    const auto normalized_gravity_base = quatBase.RotateVectorReverse(normalized_gravity_world);
    msg_ref.normalized_gravity_b = {normalized_gravity_base.X(), normalized_gravity_base.Y(),
                                    normalized_gravity_base.Z()};

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
        scalar_t current_angle = msg_ref.joint_pos[i];
        scalar_t last_angle = lastJointAngles_[i];
        scalar_t velocity = (current_angle - last_angle) / dt;
        msg_ref.joint_vel[i] = velocity;
        lastJointAngles_[i] = current_angle;
    }

    // compute forward kinematics
    VectorX q(model_.nq);
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

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
Vector3 StateEstimator::mat2aa(const Matrix3 &R) {
    AngleAxis aa(R);
    return aa.axis() * aa.angle();
}

GZ_REGISTER_MODEL_PLUGIN(StateEstimator);
}  // namespace gazebo