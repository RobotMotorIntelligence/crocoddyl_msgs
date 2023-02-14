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
#include <whole_body_state_msgs/WholeBodyState.h>

#include <boost/python.hpp>

// #include "pinocchio_pybind11_compatibility.hpp"

namespace crocoddyl_msgs {

class WholeBodyStateSubscriber {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  WholeBodyStateSubscriber(pinocchio::Model &m)
      : WholeBodyStateSubscriber(m, "/crocoddyl/whole_body_state", "odom") {}

  WholeBodyStateSubscriber(pinocchio::Model &m, const std::string &topic,
                           const std::string &frame_id)
      : spinner_(2), t_(0.), q_(Eigen::VectorXd::Zero(m.nq)),
        v_(Eigen::VectorXd::Zero(m.nv)), a_(Eigen::VectorXd::Zero(m.nv)),
        tau_(Eigen::VectorXd(m.njoints - 2)), has_new_msg_(false),
        is_processing_msg_(false), last_msg_time_(0.), odom_frame_(frame_id),
        model_(m), data_(m) {
    ros::NodeHandle n;
    sub_ = n.subscribe<whole_body_state_msgs::WholeBodyState>(
        topic, 1, &WholeBodyStateSubscriber::callback, this,
        ros::TransportHints().tcpNoDelay());
    spinner_.start();

    std::cout << "Ready to receive whole-body states" << std::endl;
  }

  ~WholeBodyStateSubscriber() = default;

  /**
   * @brief Get the latest whole-body state
   *
   * @return  A tuple with the vector of time at the beginning of the interval,
   * generalized position, generalized velocity, joint efforts, and contact
   * state.
   */
  std::tuple<
      double, Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd,
      std::map<std::string, pinocchio::SE3>,
      std::map<std::string, pinocchio::Motion>,
      std::map<std::string, std::tuple<pinocchio::Force, uint8_t, uint8_t>>,
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
    return {t_, q_, v_, tau_, p_, pd_, f_, s_};
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
  Eigen::VectorXd tau_; ///< Torque vector (size njoints-2)
  std::map<std::string, pinocchio::SE3> p_;     //!< Contact position
  std::map<std::string, pinocchio::Motion> pd_; //!< Contact velocity
  std::map<std::string, std::tuple<pinocchio::Force, uint8_t, uint8_t>>
      f_; //!< Contact force, type and status
  std::map<std::string, std::pair<Eigen::Vector3d, double>>
      s_;                  //!< Contact surface and friction coefficient
  bool has_new_msg_;       //!< Indcate when a new message has been received
  bool is_processing_msg_; //!< Indicate when we are processing the message
  double last_msg_time_ = 0;
  std::string odom_frame_;
  pinocchio::Model model_;
  pinocchio::Data data_;
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