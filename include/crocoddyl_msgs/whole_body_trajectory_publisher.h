///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2023-2023, Heriot-Watt University
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef CROCODDYL_MSG_WHOLE_BODY_TRAJECTORY_PUBLISHER_H_
#define CROCODDYL_MSG_WHOLE_BODY_TRAJECTORY_PUBLISHER_H_

#include "crocoddyl_msgs/conversions.h"

#include <realtime_tools/realtime_publisher.h>

#ifdef ROS2
#include <rclcpp/rclcpp.hpp>
#else
#include <ros/node_handle.h>
#endif

namespace crocoddyl_msgs {

class WholeBodyTrajectoryRosPublisher {
public:
  /**
   * @brief Initialize the whole-body trajectory publisher.
   *
   * @param[in] model  Pinocchio model
   * @param[in] topic  Topic name
   * @param[in] frame  Odometry frame
   * @param[in] queue  Queue size
   */
  WholeBodyTrajectoryRosPublisher(
      pinocchio::Model &model,
      const std::string &topic = "/crocoddyl/whole_body_trajectory",
      const std::string &frame = "odom", int queue = 10)
#ifdef ROS2
      : node_("whole_body_trajectory_publisher"),
        pub_(node_.create_publisher<WholeBodyTrajectory>(topic, queue)),
        model_(model), data_(model), odom_frame_(frame), a_(model.nv),
        is_reduced_model_(false) {
    RCLCPP_INFO_STREAM(node_.get_logger(),
                       "Publishing WholeBodyTrajectory messages on "
                           << topic << " (frame: " << frame << ")");
#else
      : model_(model), data_(model), odom_frame_(frame), a_(model.nv),
        is_reduced_model_(false) {
    ros::NodeHandle n;
    pub_.init(n, topic, queue);
    ROS_INFO_STREAM("Publishing WholeBodyTrajectory messages on "
                    << topic << " (frame: " << frame << ")");
#endif
    pub_.msg_.header.frame_id = frame;
    init();
  }

  /**
   * @brief Initialize the whole-body trajectory publisher for rigid-body system
   * with locked joints.
   *
   * @param[in] model          Pinocchio model
   * @param[in] locked_joints  List of joints to be locked
   * @param[in] qref           Reference configuration
   * @param[in] topic          Topic name
   * @param[in] frame          Odometry frame
   * @param[in] queue          Queue size
   */
  WholeBodyTrajectoryRosPublisher(
      pinocchio::Model &model, const std::vector<std::string> &locked_joints,
      const Eigen::Ref<const Eigen::VectorXd> &qref,
      const std::string &topic = "/crocoddyl/whole_body_trajectory",
      const std::string &frame = "odom", int queue = 10)
#ifdef ROS2
      : node_("whole_body_trajectory_publisher"),
        pub_(node_.create_publisher<WholeBodyTrajectory>(topic, queue)),
        model_(model), data_(model), odom_frame_(frame), a_(model.nv),
        qref_(qref), is_reduced_model_(true) {
    RCLCPP_INFO_STREAM(node_.get_logger(),
                       "Publishing WholeBodyTrajectory messages on "
                           << topic << " (frame: " << frame << ")");
#else
      : model_(model), data_(model), odom_frame_(frame), a_(model.nv),
        qref_(qref), is_reduced_model_(true) {
    ros::NodeHandle n;
    pub_.init(n, topic, queue);
    ROS_INFO_STREAM("Publishing WholeBodyTrajectory messages on "
                    << topic << " (frame: " << frame << ")");
#endif
    init(locked_joints);
  }

  ~WholeBodyTrajectoryRosPublisher() = default;

  /**
   * @brief Publish a whole-body trajectory ROS message.
   * The dimension of the state and control is defined by the rigid-body system
   * with locked joints.
   *
   * @param ts[in]   Vector of interval times in secs
   * @param xs[in]   Vector of states (dimension: model.nq - locked_joints)
   * @param us[in]   Vector of joint efforts (dimension: model.nv -
   * locked_joints)
   * @param ps[in]   Vector of contact positions
   * @param pds[in]  Vector of contact velocities
   * @param fs[in]   Vector of contact forces, types and statuses
   * @param ss[in]   Vector of contact surfaces and friction coefficients
   */
  void
  publish(const std::vector<double> &ts, const std::vector<Eigen::VectorXd> &xs,
          const std::vector<Eigen::VectorXd> &us,
          const std::vector<std::map<std::string, pinocchio::SE3>> &ps,
          const std::vector<std::map<std::string, pinocchio::Motion>> &pds,
          const std::vector<
              std::map<std::string, std::tuple<pinocchio::Force, ContactType,
                                               ContactStatus>>> &fs,
          const std::vector<
              std::map<std::string, std::pair<Eigen::Vector3d, double>>> &ss) {
    if (pub_.trylock()) {
      if (ts.size() != xs.size()) {
        throw std::invalid_argument("The size of the ts vector needs to equal "
                                    "the size of the xs vector.");
      }
      if (ts.size() != us.size()) {
        throw std::invalid_argument(
            "If provided, the size of the us vector needs to equal the size of "
            "the ts vector.");
      }
      if (ts.size() != ps.size()) {
        throw std::invalid_argument(
            "If provided, the size of the ps vector needs to equal the size of "
            "the ts vector.");
      }
      if (ts.size() != pds.size()) {
        throw std::invalid_argument("If provided, the size of the pds vector "
                                    "needs to equal the size of "
                                    "the ts vector.");
      }
      if (ts.size() != fs.size()) {
        throw std::invalid_argument(
            "If provided, the size of the fs vector needs to equal the size of "
            "the ts vector.");
      }
      if (ts.size() != ss.size()) {
        throw std::invalid_argument(
            "If provided, the size of the ss vector needs to equal the size of "
            "the ts vector.");
      }
      pub_.msg_.header.frame_id = odom_frame_;
#ifdef ROS2
      pub_.msg_.header.stamp = node_.now();
#else
      pub_.msg_.header.stamp = ros::Time::now();
#endif
      pub_.msg_.trajectory.resize(ts.size());
      for (std::size_t i = 0; i < ts.size(); ++i) {
        pub_.msg_.trajectory[i].header.frame_id = odom_frame_;
        if (is_reduced_model_) {
          fromReduced(model_, reduced_model_, qfull_, vfull_, ufull_,
                      xs[i].head(reduced_model_.nq),
                      xs[i].tail(reduced_model_.nv), us[i], qref_, joint_ids_);
          crocoddyl_msgs::toMsg(model_, data_, pub_.msg_.trajectory[i],
                                ts[i], qfull_, vfull_, a_, ufull_, ps[i],
                                pds[i], fs[i], ss[i]);
        } else {
          crocoddyl_msgs::toMsg(model_, data_, pub_.msg_.trajectory[i], ts[i],
                                xs[i].head(model_.nq), xs[i].tail(model_.nv),
                                a_, us[i], ps[i], pds[i], fs[i], ss[i]);
        }
      }
      pub_.unlockAndPublish();
    } else {
#ifdef ROS2
      RCLCPP_WARN_STREAM(
          node_.get_logger(),
          "[publish] Could not lock publisher, not published feedback policy");
#else
      ROS_WARN_STREAM(
          "[publish] Could not lock publisher, not published feedback policy");
#endif
    }
  }

private:
#ifdef ROS2
  rclcpp::Node node_;
#endif
  realtime_tools::RealtimePublisher<WholeBodyTrajectory> pub_;
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

  void init(const std::vector<std::string> &locked_joints = DEFAULT_VECTOR) {
    a_.setZero();

    if (locked_joints.size() != 0) {
      // Check the size of the reference configuration
#ifdef ROS2
      if (qref_.size() != model_.nq) {
        RCLCPP_ERROR_STREAM(node_.get_logger(), "Invalid argument: qref has wrong dimension (it should be " << std::to_string(model_.nq) << ")";
      }
#else
      if (qref_.size() != model_.nq) {
        ROS_ERROR_STREAM(
            "Invalid argument: qref has wrong dimension (it should be "
            << std::to_string(model_.nq) << ")");
      }
#endif
      // Build the reduced model
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

      // Initialize the vectors and dimensions
      const std::size_t root_joint_id = getRootJointId(model_);
      const std::size_t nv_root = model_.joints[root_joint_id].nv();
      qfull_ = Eigen::VectorXd::Zero(model_.nq);
      vfull_ = Eigen::VectorXd::Zero(model_.nv);
      ufull_ = Eigen::VectorXd::Zero(model_.nv - nv_root);
    }
  }
};

} // namespace crocoddyl_msgs

#endif // CROCODDYL_MSG_WHOLE_BODY_TRAJECTORY_PUBLISHER_H_
