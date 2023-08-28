///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2023-2023, Heriot-Watt University
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef CROCODDYL_MSG_WHOLE_BODY_TRAJECTORY_SUBSCRIBER_H_
#define CROCODDYL_MSG_WHOLE_BODY_TRAJECTORY_SUBSCRIBER_H_

#include "crocoddyl_msgs/conversions.h"

#include <mutex>
#ifdef ROS2
#include <rclcpp/rclcpp.hpp>
#else
#include <ros/node_handle.h>
#endif

namespace crocoddyl_msgs {

#ifdef ROS2
typedef const whole_body_state_msgs::msg::WholeBodyTrajectory::SharedPtr
    WholeBodyTrajectorySharedPtr;
#else
typedef const whole_body_state_msgs::WholeBodyTrajectory::ConstPtr
    &WholeBodyTrajectorySharedPtr;
#endif

class WholeBodyTrajectoryRosSubscriber {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Initialize the whole-body state subscriber
   *
   * @param[in] model  Pinocchio model
   * @param[in] topic  Topic name
   * @param[in] frame  Odometry frame
   */
#ifdef ROS2
  WholeBodyTrajectoryRosSubscriber(
      pinocchio::Model &model,
      const std::string &topic = "/crocoddyl/whole_body_trajectory",
      const std::string &frame = "odom")
      : node_(rclcpp::Node::make_shared("whole_body_trajectory_subscriber")),
        sub_(node_->create_subscription<WholeBodyTrajectory>(
            topic, 1,
            std::bind(&WholeBodyTrajectoryRosSubscriber::callback, this,
                      std::placeholders::_1))),
        has_new_msg_(false), is_processing_msg_(false), last_msg_time_(0.),
        odom_frame_(frame), model_(model), data_(model), a_(model.nv),
        is_reduced_model_(false) {
    spinner_.add_node(node_);
    thread_ = std::thread([this]() { this->spin(); });
    thread_.detach();
    RCLCPP_INFO_STREAM(node_->get_logger(),
                       "Subscribing WholeBodyTrajectory messages on " << topic);
#else
  WholeBodyTrajectoryRosSubscriber(
      pinocchio::Model &model,
      const std::string &topic = "/crocoddyl/whole_body_trajectory",
      const std::string &frame = "odom")
      : spinner_(2), has_new_msg_(false), is_processing_msg_(false),
        last_msg_time_(0.), odom_frame_(frame), model_(model), data_(model),
        a_(model.nv), is_reduced_model_(false) {
    ros::NodeHandle n;
    sub_ = n.subscribe<WholeBodyTrajectory>(
        topic, 1, &WholeBodyTrajectoryRosSubscriber::callback, this,
        ros::TransportHints().tcpNoDelay());
    spinner_.start();
    ROS_INFO_STREAM("Subscribing WholeBodyTrajectory messages on " << topic);
#endif
    a_.setZero();
    const std::size_t root_joint_id = get_root_joint_id(model);
    nx_ = model_.nq + model_.nv;
    nu_ = model.nv - model.joints[root_joint_id].nv();
  }

  /**
   * @brief Initialize the whole-body state subscriber for rigid-body system
   * with locked joints.
   *
   * @param[in] model          Pinocchio model
   * @param[in] locked_joints  List of joints to be locked
   * @param[in] qref           Reference configuration
   * @param[in] topic          Topic name
   * @param[in] frame          Odometry frame
   */
#ifdef ROS2
  WholeBodyTrajectoryRosSubscriber(
      pinocchio::Model &model, std::vector<std::string> locked_joints,
      const Eigen::Ref<const Eigen::VectorXd> &qref,
      const std::string &topic = "/crocoddyl/whole_body_trajectory",
      const std::string &frame = "odom")
      : node_(rclcpp::Node::make_shared("whole_body_trajectory_subscriber")),
        sub_(node_->create_subscription<WholeBodyTrajectory>(
            topic, 1,
            std::bind(&WholeBodyTrajectoryRosSubscriber::callback, this,
                      std::placeholders::_1))),
        has_new_msg_(false), is_processing_msg_(false), last_msg_time_(0.),
        odom_frame_(frame), model_(model), data_(model), a_(model.nv),
        qref_(qref), is_reduced_model_(false) {
    spinner_.add_node(node_);
    thread_ = std::thread([this]() { this->spin(); });
    thread_.detach();
    RCLCPP_INFO_STREAM(node_->get_logger(),
                       "Subscribing WholeBodyTrajectory messages on " << topic);
    if (qref_.size() != model_.nq) {
      RCLCPP_ERROR_STREAM(node_.get_logger(), "Invalid argument: qref has wrong dimension (it should be " << std::to_string(model_.nq) << ")";
    }
#else
  WholeBodyTrajectoryRosSubscriber(
      pinocchio::Model &model, std::vector<std::string> locked_joints,
      const Eigen::Ref<const Eigen::VectorXd> &qref,
      const std::string &topic = "/crocoddyl/whole_body_trajectory",
      const std::string &frame = "odom")
      : spinner_(2), has_new_msg_(false), is_processing_msg_(false),
        last_msg_time_(0.), odom_frame_(frame), model_(model), data_(model),
        a_(model.nv - locked_joints.size()), qref_(qref),
        is_reduced_model_(false) {
    ros::NodeHandle n;
    sub_ = n.subscribe<WholeBodyTrajectory>(
        topic, 1, &WholeBodyTrajectoryRosSubscriber::callback, this,
        ros::TransportHints().tcpNoDelay());
    spinner_.start();
    ROS_INFO_STREAM("Subscribing WholeBodyTrajectory messages on " << topic);
    if (qref_.size() != model_.nq) {
      ROS_ERROR_STREAM(
          "Invalid argument: qref has wrong dimension (it should be "
          << std::to_string(model_.nq) << ")");
    }
#endif
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
    a_.setZero();
    qfull_ = Eigen::VectorXd::Zero(model_.nq);
    vfull_ = Eigen::VectorXd::Zero(model_.nv);
    ufull_ = Eigen::VectorXd::Zero(model_.nv - nv_root);
    nx_ = reduced_model_.nq + reduced_model_.nv;
    nu_ = reduced_model_.nv - nv_root;
  }
  ~WholeBodyTrajectoryRosSubscriber() = default;

  /**
   * @brief Get the latest whole-body trajectory for the rigid-body system with
   * locked joints.
   *
   * @todo: Use Pinocchio objects once there is a pybind11 support of std
   * containers.
   *
   * @return  A tuple with the vector of time at the beginning of the
   * interval, state vector, joint efforts, contact positions, contact
   * velocities, contact forces (wrench, type and status), and contact surface
   * and friction coefficients.
   */
  std::tuple<
      std::vector<double>, std::vector<Eigen::VectorXd>,
      std::vector<Eigen::VectorXd>,
      std::vector<
          std::map<std::string, std::pair<Eigen::Vector3d, Eigen::MatrixXd>>>,
      std::vector<std::map<std::string, Eigen::VectorXd>>,
      std::vector<std::map<std::string, std::tuple<Eigen::VectorXd, ContactType,
                                                   ContactStatus>>>,
      std::vector<std::map<std::string, std::pair<Eigen::Vector3d, double>>>>
  get_trajectory() {
    // start processing the message
    is_processing_msg_ = true;
    std::lock_guard<std::mutex> guard(mutex_);

    const std::size_t N = msg_.trajectory.size();
    ts_.resize(N);
    xs_.resize(N);
    us_.resize(N);
    ps_.resize(N);
    pds_.resize(N);
    fs_.resize(N);
    ss_.resize(N);
    for (std::size_t i = 0; i < N; ++i) {
      xs_[i].resize(nx_);
      us_[i].resize(nu_);
      if (is_reduced_model_) {
        crocoddyl_msgs::fromMsg(model_, data_, msg_.trajectory[i], ts_[i],
                                qfull_, vfull_, a_, ufull_, ps_[i], pds_[i],
                                fs_[i], ss_[i]);
        toReduced(model_, reduced_model_, xs_[i].head(reduced_model_.nq),
                  xs_[i].tail(reduced_model_.nv), us_[i], qfull_, vfull_,
                  ufull_);

      } else {
        crocoddyl_msgs::fromMsg(model_, data_, msg_.trajectory[i], ts_[i],
                                xs_[i].head(model_.nq), xs_[i].tail(model_.nv),
                                a_, us_[i], ps_[i], pds_[i], fs_[i], ss_[i]);
      }
      // create maps that do not depend on Pinocchio objects
      ps_tmp_.resize(N);
      pds_tmp_.resize(N);
      fs_tmp_.resize(N);
      for (auto it = ps_[i].cbegin(); it != ps_[i].cend(); ++it) {
        p_tmp_[it->first] =
            std::make_pair(it->second.translation(), it->second.rotation());
      }
      ps_tmp_[i] = p_tmp_;
      for (auto it = pds_[i].cbegin(); it != pds_[i].cend(); ++it) {
        pd_tmp_[it->first] = it->second.toVector();
      }
      pds_tmp_[i] = pd_tmp_;
      for (auto it = fs_[i].cbegin(); it != fs_[i].cend(); ++it) {
        f_tmp_[it->first] =
            std::make_tuple(std::get<0>(it->second).toVector(),
                            std::get<1>(it->second), std::get<2>(it->second));
      }
      fs_tmp_[i] = f_tmp_;
    }
    // finish processing the message
    is_processing_msg_ = false;
    has_new_msg_ = false;
    return {ts_, xs_, us_, ps_tmp_, pds_tmp_, fs_tmp_, ss_};
  }

  /**
   * @brief Indicate whether we have received a new message
   */
  bool has_new_msg() const { return has_new_msg_; }

private:
#ifdef ROS2
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::executors::SingleThreadedExecutor spinner_;
  std::thread thread_;
  void spin() { spinner_.spin(); }
  rclcpp::Subscription<WholeBodyTrajectory>::SharedPtr sub_; //!< ROS subscriber
#else
  ros::AsyncSpinner spinner_;
  ros::Subscriber sub_; //!< ROS subscriber
#endif
  std::mutex mutex_;        ///< Mutex to prevent race condition on callback
  WholeBodyTrajectory msg_; //!< ROS message
  std::vector<double> ts_;  //!< Vector of time at the beginning of the interval
  std::vector<Eigen::VectorXd> xs_; ///< Vector of states
  std::vector<Eigen::VectorXd> us_; ///< Vector of joint efforts
  std::vector<std::map<std::string, pinocchio::SE3>>
      ps_; //!< Vector of contact positions
  std::vector<std::map<std::string, pinocchio::Motion>>
      pds_; //!< Vector of contact velocities
  std::vector<std::map<
      std::string, std::tuple<pinocchio::Force, ContactType, ContactStatus>>>
      fs_; //!< Vector of contact forces, types and statuses
  std::vector<std::map<std::string, std::pair<Eigen::Vector3d, double>>>
      ss_;           //!< Vector of contact surfaces and friction coefficients
  bool has_new_msg_; //!< Indcate when a new message has been received
  bool is_processing_msg_; //!< Indicate when we are processing the message
  double last_msg_time_;
  std::string odom_frame_;
  pinocchio::Model model_;
  pinocchio::Model reduced_model_;
  pinocchio::Data data_;
  Eigen::VectorXd a_;
  Eigen::VectorXd
      qref_; ///!< Reference configuration used in the reduced model (size nq)
  std::vector<pinocchio::JointIndex> joint_ids_; ///!< List of locked joint Ids
  Eigen::VectorXd qfull_; ///!< Full configuration vector (size nq)
  Eigen::VectorXd vfull_; ///!< Full tangent vector (size nv)
  Eigen::VectorXd ufull_; ///!< Full torque vector (size njoints)
  bool is_reduced_model_; ///< True if we have reduced the model
  std::size_t nx_;
  std::size_t nu_;
  // TODO(cmastalli): Temporal variables as it is needed std container support
  // in Pinocchio
  std::map<std::string, std::pair<Eigen::Vector3d, Eigen::MatrixXd>> p_tmp_;
  std::map<std::string, Eigen::VectorXd> pd_tmp_;
  std::map<std::string, std::tuple<Eigen::VectorXd, ContactType, ContactStatus>>
      f_tmp_;
  std::vector<
      std::map<std::string, std::pair<Eigen::Vector3d, Eigen::MatrixXd>>>
      ps_tmp_;
  std::vector<std::map<std::string, Eigen::VectorXd>> pds_tmp_;
  std::vector<std::map<std::string,
                       std::tuple<Eigen::VectorXd, ContactType, ContactStatus>>>
      fs_tmp_;
  void callback(WholeBodyTrajectorySharedPtr msg) {
    if (msg->header.frame_id != odom_frame_) {
#ifdef ROS2
      RCLCPP_ERROR_STREAM(
          node_->get_logger(),
          "Error: the whole-body trajectory is not expressed in "
              << odom_frame_ << " (i.e., 'odom_frame')");
#else
      ROS_ERROR_STREAM("Error: the whole-body trajectory is not expressed in "
                       << odom_frame_ << " (i.e., 'odom_frame')");
#endif
      return;
    }
    if (!is_processing_msg_) {
#ifdef ROS2
      double t = rclcpp::Time(msg->header.stamp).seconds();
#else
      double t = msg->header.stamp.toNSec();
#endif
      // Avoid out of order arrival and ensure each message is newer (or equal
      // to) than the preceeding:
      if (last_msg_time_ <= t) {
        std::lock_guard<std::mutex> guard(mutex_);
        msg_ = *msg;
        has_new_msg_ = true;
        last_msg_time_ = t;
      } else {
#ifdef ROS2
        RCLCPP_WARN_STREAM(node_->get_logger(),
                           "Out of order message. Last timestamp: "
                               << std::fixed << last_msg_time_
                               << ", current timestamp: " << t);
#else
        ROS_WARN_STREAM("Out of order message. Last timestamp: "
                        << std::fixed << last_msg_time_
                        << ", current timestamp: " << t);
#endif
      }
    }
  }
};

} // namespace crocoddyl_msgs

#endif // CROCODDYL_MSG_WHOLE_BODY_TRAJECTORY_SUBSCRIBER_H_
