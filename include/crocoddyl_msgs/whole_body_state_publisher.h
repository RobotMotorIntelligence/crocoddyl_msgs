///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2023-2023, Heriot-Watt University
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef CROCODDYL_MSG_WHOLE_BODY_STATE_PUBLISHER_H_
#define CROCODDYL_MSG_WHOLE_BODY_STATE_PUBLISHER_H_

#include "crocoddyl_msgs/conversions.h"

#include <realtime_tools/realtime_publisher.h>

#ifdef ROS2
#include <rclcpp/rclcpp.hpp>
#else
#include <ros/node_handle.h>
#endif

namespace crocoddyl_msgs {

class WholeBodyStateRosPublisher {
public:
  /**
   * @brief Initialize the whole-body state publisher.
   *
   * @param[in] model  Pinocchio model
   * @param[in] topic  Topic name
   * @param[in] frame  Odometry frame
   */
#ifdef ROS2
  WholeBodyStateRosPublisher(
      pinocchio::Model &model,
      const std::string &topic = "/crocoddyl/whole_body_state",
      const std::string &frame = "odom")
      : node_("whole_body_state_publisher"),
        pub_(node_.create_publisher<WholeBodyState>(topic, 1)), model_(model),
        data_(model), odom_frame_(frame), a_(model.nv),
        is_reduced_model_(false) {
    RCLCPP_INFO_STREAM(node_.get_logger(),
                       "Publishing WholeBodyState messages on "
                           << topic << " (frame: " << frame << ")");
#else
  WholeBodyStateRosPublisher(
      pinocchio::Model &model,
      const std::string &topic = "/crocoddyl/whole_body_state",
      const std::string &frame = "odom")
      : model_(model), data_(model), odom_frame_(frame), a_(model.nv),
        is_reduced_model_(false) {
    ros::NodeHandle n;
    pub_.init(n, topic, 1);
    ROS_INFO_STREAM("Publishing WholeBodyState messages on "
                    << topic << " (frame: " << frame << ")");
#endif
    a_.setZero();
    pub_.msg_.header.frame_id = frame;
  }

  /**
   * @brief Initialize the whole-body state publisher for rigid-body system
   * with locked joints.
   *
   * @param[in] model          Pinocchio model
   * @param[in] locked_joints  List of joints to be locked
   * @param[in] qref           Reference configuration
   * @param[in] topic          Topic name
   * @param[in] frame          Odometry frame
   */
#ifdef ROS2
  WholeBodyStateRosPublisher(
      pinocchio::Model &model, std::vector<std::string> locked_joints,
      const Eigen::Ref<const Eigen::VectorXd> &qref,
      const std::string &topic = "/crocoddyl/whole_body_state",
      const std::string &frame = "odom")
      : node_("whole_body_state_publisher"),
        pub_(node_.create_publisher<WholeBodyState>(topic, 1)), model_(model),
        odom_frame_(frame), a_(model.nv - locked_joints.size()), qref_(qref),
        is_reduced_model_(true) {
    RCLCPP_INFO_STREAM(node_.get_logger(),
                       "Publishing WholeBodyState messages on "
                           << topic << " (frame: " << frame << ")");
    if (qref_.size() != model_.nq) {
      RCLCPP_ERROR_STREAM(node_.get_logger(), "Invalid argument: qref has wrong dimension (it should be " << std::to_string(model_.nq) << ")";
    }
#else
  WholeBodyStateRosPublisher(
      pinocchio::Model &model, std::vector<std::string> locked_joints,
      const Eigen::Ref<const Eigen::VectorXd> &qref,
      const std::string &topic = "/crocoddyl/whole_body_state",
      const std::string &frame = "odom")
      : model_(model), odom_frame_(frame), a_(model.nv - locked_joints.size()),
        qref_(qref), is_reduced_model_(true) {
    ros::NodeHandle n;
    pub_.init(n, topic, 1);
    ROS_INFO_STREAM("Publishing WholeBodyState messages on "
                    << topic << " (frame: " << frame << ")");
    if (qref_.size() != model_.nq) {
      ROS_ERROR_STREAM(
          "Invalid argument: qref has wrong dimension (it should be "
          << std::to_string(model_.nq) << ")");
    }
#endif
    a_.setZero();
    pub_.msg_.header.frame_id = frame;

    // Build reduce model
    for (std::string name : locked_joints) {
      if (model_.existJointName(name)) {
        joint_ids_.push_back(model_.getJointId(name));
      } else {
#ifdef ROS2
        RCLCPP_ERROR_STREAM(node_.get_logger(),
                            "Doesn't exist " << name << " joint");
#else
        ROS_ERROR_STREAM("Doesn't exist " << name << " joint");
#endif
      }
    }
    pinocchio::buildReducedModel(model_, joint_ids_, qref_, reduced_model_);
    data_ = pinocchio::Data(reduced_model_);

    const std::size_t root_joint_id = get_root_joint_id(model);
    const std::size_t nv_root = model.joints[root_joint_id].nv();
    qfull_ = Eigen::VectorXd::Zero(model.nq);
    vfull_ = Eigen::VectorXd::Zero(model.nv);
    ufull_ = Eigen::VectorXd::Zero(model.nv - nv_root);
  }
  ~WholeBodyStateRosPublisher() = default;

  /**
   * @brief Publish a whole-body state ROS message.
   * The dimension of the configuration, velocity and joint effort are defined
   * by the rigid-body system with locked joints.
   *
   * @param t[in]    Time in secs
   * @param q[in]    Configuration vector (dimension: model.nq)
   * @param v[in]    Generalized velocity (dimension: model.nv)
   * @param tau[in]  Joint effort (dimension: model.nv)
   * @param p[in]    Contact position
   * @param pd[in]   Contact velocity
   * @param f[in]    Contact force, type and status
   * @param s[in]    Contact surface and friction coefficient
   */
  void
  publish(const double t, const Eigen::Ref<const Eigen::VectorXd> &q,
          const Eigen::Ref<const Eigen::VectorXd> &v,
          const Eigen::Ref<const Eigen::VectorXd> &tau,
          const std::map<std::string, pinocchio::SE3> &p,
          const std::map<std::string, pinocchio::Motion> &pd,
          const std::map<std::string, std::tuple<pinocchio::Force, ContactType,
                                                 ContactStatus>> &f,
          const std::map<std::string, std::pair<Eigen::Vector3d, double>> &s) {
    if (pub_.trylock()) {
      pub_.msg_.header.frame_id = odom_frame_;
      if (is_reduced_model_) {
        fromReduced(model_, reduced_model_, qfull_, vfull_, ufull_, q, v, tau,
                    qref_, joint_ids_);
        crocoddyl_msgs::toMsg(reduced_model_, data_, pub_.msg_, t, qfull_,
                              vfull_, a_, ufull_, p, pd, f, s);
      } else {
        crocoddyl_msgs::toMsg(model_, data_, pub_.msg_, t, q, v, a_, tau, p, pd,
                              f, s);
      }
      pub_.unlockAndPublish();
    }
  }

private:
#ifdef ROS2
  rclcpp::Node node_;
#endif
  realtime_tools::RealtimePublisher<WholeBodyState> pub_;
  pinocchio::Model model_;
  pinocchio::Model reduced_model_;
  pinocchio::Data data_;
  std::string odom_frame_;
  Eigen::VectorXd a_;
  std::vector<pinocchio::JointIndex> joint_ids_;
  Eigen::VectorXd qref_;
  Eigen::VectorXd qfull_;
  Eigen::VectorXd vfull_;
  Eigen::VectorXd ufull_;
  bool is_reduced_model_;
};

} // namespace crocoddyl_msgs

#endif // CROCODDYL_MSG_WHOLE_BODY_STATE_PUBLISHER_H_
