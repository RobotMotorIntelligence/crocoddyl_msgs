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
      : spinner_(2), state_(m.nq, m.nv, m.njoints - 2), has_new_msg_(false),
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

  // TODO: Change Vector6d to pinocchio::Force
  //       Not urgent, since it's unused in the MPC right now.
  /**
   * @brief Get the latest whole-body state
   *
   * @return  A tuple with the vector of time at the beginning of the interval,
   * generalized position, generalized velocity, joint efforts, and contact
   * state.
   */
  std::tuple<
      double, Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd,
      std::map<std::string,
               std::tuple<uint8_t, Eigen::Matrix<double, 6, -1>, uint8_t>>>
  get_state() {
    // start processing the message
    is_processing_msg_ = true;
    std::lock_guard<std::mutex> guard(mutex_);
    crocoddyl_msgs::fromMsg(model_, data_, msg_, state_.t, state_.q, state_.v,
                            state_.a, state_.tau, state_.contacts);
    for (const auto &contact : state_.contacts) {
      contacts_[contact.first] = {contact.second.type,
                                  contact.second.force.toVector(),
                                  contact.second.state};
    }
    // finish processing the message
    is_processing_msg_ = false;
    has_new_msg_ = false;
    return {state_.t, state_.q, state_.v, state_.tau, contacts_};
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
  WholeBodyState state_;                      //!< Whole-body state
  std::map<std::string, std::tuple<uint8_t, Eigen::Matrix<double, 6, -1>,
                                   uint8_t>> //!< Contact state
      contacts_;
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