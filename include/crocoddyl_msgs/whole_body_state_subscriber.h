///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2020-2023, Heriot-Watt University, University of Oxford
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef CROCODDYL_MSG_WHOLE_BODY_STATE_SUBSCRIBER_H_
#define CROCODDYL_MSG_WHOLE_BODY_STATE_SUBSCRIBER_H_

#include "crocoddyl_msgs/conversions.h"

#include <mutex>
#include <ros/node_handle.h>

namespace crocoddyl_msgs {

class WholeBodyStateRosSubscriber {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Initialize the whole-body state subscriber
   *
   * @param[in] model  Pinocchio model
   * @param[in] topic  Topic name
   * @param[in] frame  Odometry frame
   */
  WholeBodyStateRosSubscriber(
      pinocchio::Model &model,
      const std::string &topic = "/crocoddyl/whole_body_state",
      const std::string &frame = "odom")
      : spinner_(2), t_(0.), q_(model.nq), v_(model.nv), a_(model.nv),
        has_new_msg_(false), is_processing_msg_(false), last_msg_time_(0.),
        odom_frame_(frame), model_(model), data_(model) {
    ros::NodeHandle n;
    sub_ = n.subscribe<whole_body_state_msgs::WholeBodyState>(
        topic, 1, &WholeBodyStateRosSubscriber::callback, this,
        ros::TransportHints().tcpNoDelay());
    spinner_.start();

    const std::size_t root_joint_id = model.frames[1].parent;
    const std::size_t nv_root = model.joints[root_joint_id].idx_q() == 0
                                    ? model.joints[root_joint_id].nv()
                                    : 0;
    const std::size_t njoints = model.nv - nv_root;
    q_.setZero();
    v_.setZero();
    a_.setZero();
    tau_ = Eigen::VectorXd::Zero(njoints);
    std::cout << "Ready to receive whole-body states" << std::endl;
  }
  ~WholeBodyStateRosSubscriber() = default;

  /**
   * @brief Get the latest whole-body state
   *
   * @todo: Use Pinocchio objects once there is a pybind11 support of std
   * containers
   *
   * @return  A tuple with the time at the beginning of the interval,
   * generalized position, generalized velocity, joint efforts, contact
   * position, contact velocity, contact force (wrench, type, status), and
   * contact surface (norm and friction coefficient).
   */
  std::tuple<double, Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd,
             std::map<std::string, std::pair<Eigen::Vector3d, Eigen::MatrixXd>>,
             std::map<std::string, Eigen::VectorXd>,
             std::map<std::string,
                      std::tuple<Eigen::VectorXd, ContactType, ContactStatus>>,
             std::map<std::string, std::pair<Eigen::Vector3d, double>>>
  get_state() {
    // start processing the message
    is_processing_msg_ = true;
    std::lock_guard<std::mutex> guard(mutex_);
    crocoddyl_msgs::fromMsg(model_, data_, msg_, t_, q_, v_, a_, tau_, p_, pd_,
                            f_, s_);
    // finish processing the message
    is_processing_msg_ = false;
    has_new_msg_ = false;
    // create maps that do not depend on Pinocchio objects
    for (auto it = p_.cbegin(); it != p_.cend(); ++it) {
      p_tmp_[it->first] =
          std::make_pair(it->second.translation(), it->second.rotation());
    }
    for (auto it = pd_.cbegin(); it != pd_.cend(); ++it) {
      pd_tmp_[it->first] = it->second.toVector();
    }
    for (auto it = f_.cbegin(); it != f_.cend(); ++it) {
      f_tmp_[it->first] =
          std::make_tuple(std::get<0>(it->second).toVector(),
                          std::get<1>(it->second), std::get<2>(it->second));
    }
    return {t_, q_, v_, tau_, p_tmp_, pd_tmp_, f_tmp_, s_};
  }

  /**
   * @brief Indicate whether we have received a new message
   */
  bool has_new_msg() const { return has_new_msg_; }

private:
  ros::AsyncSpinner spinner_;
  ros::Subscriber sub_; //!< ROS subscriber
  std::mutex mutex_;    ///< Mutex to prevent race condition on callback
  whole_body_state_msgs::WholeBodyState msg_; //!< ROS message
  double t_;            //!< Time at the beginning of the interval
  Eigen::VectorXd q_;   ///< Configuration vector (size nq)
  Eigen::VectorXd v_;   ///< Tangent vector (size nv)
  Eigen::VectorXd a_;   ///< System acceleration vector (size nv)
  Eigen::VectorXd tau_; ///< Torque vector (size njoints)
  std::map<std::string, pinocchio::SE3> p_;     //!< Contact position
  std::map<std::string, pinocchio::Motion> pd_; //!< Contact velocity
  std::map<std::string,
           std::tuple<pinocchio::Force, ContactType, ContactStatus>>
      f_; //!< Contact force, type and status
  std::map<std::string, std::pair<Eigen::Vector3d, double>>
      s_;                  //!< Contact surface and friction coefficient
  bool has_new_msg_;       //!< Indcate when a new message has been received
  bool is_processing_msg_; //!< Indicate when we are processing the message
  double last_msg_time_;
  std::string odom_frame_;
  pinocchio::Model model_;
  pinocchio::Data data_;
  // TODO(cmastalli): Temporal variables as it is needed std container support
  // in Pinocchio
  std::map<std::string, std::pair<Eigen::Vector3d, Eigen::MatrixXd>> p_tmp_;
  std::map<std::string, Eigen::VectorXd> pd_tmp_;
  std::map<std::string, std::tuple<Eigen::VectorXd, ContactType, ContactStatus>>
      f_tmp_;
  void callback(const whole_body_state_msgs::WholeBodyStateConstPtr &msg) {
    if (msg->header.frame_id != odom_frame_) {
      ROS_ERROR_STREAM("Error: the whole-body state is not expressed in "
                       << odom_frame_ << " (i.e., 'odom_frame')");
      return;
    }
    if (!is_processing_msg_) {
      double t = msg->header.stamp.toNSec();
      // Avoid out of order arrival and ensure each message is newer (or equal
      // to) than the preceeding:
      if (last_msg_time_ <= t) {
        std::lock_guard<std::mutex> guard(mutex_);
        msg_ = *msg;
        has_new_msg_ = true;
        last_msg_time_ = t;
      } else {
        ROS_WARN_STREAM("Out of order message. Last timestamp: "
                        << std::fixed << last_msg_time_
                        << ", current timestamp: " << t);
      }
    }
  }
};

} // namespace crocoddyl_msgs

#endif // CROCODDYL_MSG_WHOLE_BODY_STATE_SUBSCRIBER_H_