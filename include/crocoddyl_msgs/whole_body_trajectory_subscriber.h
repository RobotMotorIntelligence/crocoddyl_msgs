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
#include <ros/node_handle.h>

namespace crocoddyl_msgs {

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
  WholeBodyTrajectoryRosSubscriber(
      pinocchio::Model &model,
      const std::string &topic = "/crocoddyl/whole_body_trajectory",
      const std::string &frame = "odom")
      : spinner_(2), a_null_(model.nv), has_new_msg_(false),
        is_processing_msg_(false), last_msg_time_(0.), odom_frame_(frame),
        model_(model), data_(model) {
    ros::NodeHandle n;
    sub_ = n.subscribe<whole_body_state_msgs::WholeBodyTrajectory>(
        topic, 1, &WholeBodyTrajectoryRosSubscriber::callback, this,
        ros::TransportHints().tcpNoDelay());
    spinner_.start();
    a_null_.setZero();
    const std::size_t root_joint_id = model_.frames[1].parent;
    const std::size_t nv_root = model_.joints[root_joint_id].idx_q() == 0
                                    ? model_.joints[root_joint_id].nv()
                                    : 0;
    nx_ = model_.nq + model_.nv;
    nu_ = model_.nv - nv_root;
    std::cout << "Ready to receive whole-body trajectory" << std::endl;
  }
  ~WholeBodyTrajectoryRosSubscriber() = default;

  /**
   * @brief Get the latest whole-body trajectory
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
      crocoddyl_msgs::fromMsg(model_, data_, msg_.trajectory[i], ts_[i],
                              xs_[i].head(model_.nq), xs_[i].tail(model_.nv),
                              a_null_, us_[i], ps_[i], pds_[i], fs_[i], ss_[i]);
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
  ros::AsyncSpinner spinner_;
  ros::Subscriber sub_; //!< ROS subscriber
  std::mutex mutex_;    ///< Mutex to prevent race condition on callback
  whole_body_state_msgs::WholeBodyTrajectory msg_; //!< ROS message
  std::vector<double> ts_; //!< Vector of time at the beginning of the interval
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
  pinocchio::Data data_;
  Eigen::VectorXd a_null_;
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
  void callback(const whole_body_state_msgs::WholeBodyTrajectoryConstPtr &msg) {
    if (msg->header.frame_id != odom_frame_) {
      ROS_ERROR_STREAM("Error: the whole-body trajectory is not expressed in "
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

#endif // CROCODDYL_MSG_WHOLE_BODY_TRAJECTORY_SUBSCRIBER_H_