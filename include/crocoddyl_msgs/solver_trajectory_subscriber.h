///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2023-2023, Heriot-Watt University
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef CROCODDYL_MSG_SOLVER_TRAJECTORY_SUBSCRIBER_H_
#define CROCODDYL_MSG_SOLVER_TRAJECTORY_SUBSCRIBER_H_

#include "crocoddyl_msgs/conversions.h"

#include <Eigen/Dense>
#include <mutex>
#include <ros/node_handle.h>

#include "crocoddyl_msgs/SolverTrajectory.h"

namespace crocoddyl_msgs {

class SolverTrajectoryRosSubscriber {
public:
  /**
   * @brief Initialize the solver trajectory subscriber
   *
   * @param[in] topic  Topic name
   */
  SolverTrajectoryRosSubscriber(
      const std::string &topic = "/crocoddyl/solver_trajectory")
      : spinner_(2), has_new_msg_(false), is_processing_msg_(false),
        last_msg_time_(0.) {
    ros::NodeHandle n;
    sub_ = n.subscribe<crocoddyl_msgs::SolverTrajectory>(
        topic, 1, &SolverTrajectoryRosSubscriber::callback, this,
        ros::TransportHints().tcpNoDelay());
    spinner_.start();
    std::cout << "Ready to subscribe to solver trajectory" << std::endl;
  }
  ~SolverTrajectoryRosSubscriber() = default;

  /**
   * @brief Get the latest solver trajectory
   *
   * @return  A tuple with the vector of time at the beginning of the interval,
   * its durations, initial state, state's rate of change, feed-forward control,
   * feedback gain, type of control and control parametrization.
   */
  std::tuple<std::vector<double>, std::vector<double>,
             std::vector<Eigen::VectorXd>, std::vector<Eigen::VectorXd>,
             std::vector<Eigen::VectorXd>, std::vector<Eigen::MatrixXd>,
             std::vector<crocoddyl_msgs::ControlType>,
             std::vector<crocoddyl_msgs::ControlParametrization>>
  get_solver_trajectory() {
    // start processing the message
    is_processing_msg_ = true;
    std::lock_guard<std::mutex> guard(mutex_);
    const std::size_t N = msg_.intervals.size();
    if (msg_.state_trajectory.size() != N) {
      throw std::invalid_argument(
          "The size of the state trajectory vector needs to equal "
          "the size of the intervals vector.");
    }
    if (msg_.control_trajectory.size() != 0 &&
        msg_.control_trajectory.size() != N) {
      throw std::invalid_argument(
          "The size of the control trajectory vector needs to equal "
          "the size of the intervals vector.");
    }

    ts_.resize(N);
    dts_.resize(N);
    xs_.resize(N);
    dxs_.resize(N);
    us_.resize(N);
    Ks_.resize(N);
    types_.resize(N);
    params_.resize(N);
    for (std::size_t i = 0; i < N; ++i) {
      ts_[i] = msg_.intervals[i].time;
      dts_[i] = msg_.intervals[i].duration;
      crocoddyl_msgs::fromMsg(msg_.state_trajectory[i], xs_[i], dxs_[i]);
      crocoddyl_msgs::fromMsg(msg_.control_trajectory[i], us_[i], Ks_[i],
                              types_[i], params_[i]);
    }
    // finish processing the message
    is_processing_msg_ = false;
    has_new_msg_ = false;
    return {ts_, dts_, xs_, dxs_, us_, Ks_, types_, params_};
  }

  /**
   * @brief Indicate whether we have received a new message
   */
  bool has_new_msg() const { return has_new_msg_; }

private:
  ros::AsyncSpinner spinner_;
  ros::Subscriber sub_; //!< ROS subscriber
  std::mutex mutex_;    //!< Mutex to prevent race condition on callback
  crocoddyl_msgs::SolverTrajectory msg_; //!< Solver trajectory message
  bool has_new_msg_;       //!< Indcate when a new message has been received
  bool is_processing_msg_; //!< Indicate when we are processing the message
  double last_msg_time_; //!< Last message time needed to ensure each message is
                         //!< newer
  std::vector<double> ts_;
  std::vector<double> dts_;
  std::vector<Eigen::VectorXd> xs_;
  std::vector<Eigen::VectorXd> dxs_;
  std::vector<Eigen::VectorXd> us_;
  std::vector<Eigen::MatrixXd> Ks_;
  std::vector<crocoddyl_msgs::ControlType> types_;
  std::vector<crocoddyl_msgs::ControlParametrization> params_;
  void callback(const crocoddyl_msgs::SolverTrajectoryConstPtr &msg) {
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

#endif // CROCODDYL_MSG_SOLVER_TRAJECTORY_SUBSCRIBER_H_