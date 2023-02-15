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
#include <ros/node_handle.h>
#include <whole_body_state_msgs/WholeBodyTrajectory.h>

namespace crocoddyl_msgs {

class WholeBodyTrajectoryRosPublisher {
public:
  /**
   * @brief Initialize the whole-body trajectory publisher
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
      : model_(model), data_(model), odom_frame_(frame), a_null_(model.nv) {
    ros::NodeHandle n;
    pub_.init(n, topic, 1);
    pub_.msg_.header.frame_id = frame;
    a_null_.setZero();
    std::cout << "Ready to send whole-body trajectory" << std::endl;
  }
  ~WholeBodyTrajectoryRosPublisher() = default;

  /**
   * @brief Publish a whole-body trajectory ROS message
   *
   * @param ts[in]      Time in secs
   * @param qs[in]      Configuration vector (dimension: model.nq)
   * @param vs[in]      Generalized velocity (dimension: model.nv)
   * @param as[in]      Generalized acceleration (dimension: model.nv)
   * @param us[in]      Joint effort (dimension: model.nv)
   * @param ps[in]      Contact position
   * @param pds[in]     Contact velocity
   * @param fs[in]      Contact force, type and status
   * @param ss[in]      Contact surface and friction coefficient
   */
  void
  publish(const std::vector<double> &ts, const std::vector<Eigen::VectorXd> &xs,
          const std::vector<Eigen::VectorXd> &us,
          const std::vector<std::map<std::string, pinocchio::SE3>> &ps,
          const std::vector<std::map<std::string, pinocchio::Motion>> &pds,
          const std::vector<std::map<
              std::string, std::tuple<pinocchio::Force, uint8_t, uint8_t>>> &fs,
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
      pub_.msg_.header.stamp = ros::Time::now();
      pub_.msg_.trajectory.resize(ts.size() - 1);
      for (std::size_t i = 0; i < ts.size(); ++i) {
        if (i == 0) {
          pub_.msg.actual.header.frame_id = odom_frame_;
          crocoddyl_msgs::toMsg(model_, data_, pub_.msg_.actual, ts[i],
                                xs[i].head(model_.nq), xs[i].tail(model_.nv),
                                a_null_, us[i], ps[i], pds[i], fs[i], ss[i]);
        } else {
          pub_.msg_.trajectory[i - 1].header.frame_id = odom_frame_;
          crocoddyl_msgs::toMsg(model_, data_, pub_.msg_.trajectory[i - 1],
                                ts[i], xs[i].head(model_.nq),
                                xs[i].tail(model_.nv), a_null_, us[i], ps[i],
                                pds[i], fs[i], ss[i]);
        }
      }
      pub_.unlockAndPublish();
    } else {
      ROS_WARN_STREAM(
          "[publish] Could not lock publisher, not published feedback policy");
    }
  }

private:
  realtime_tools::RealtimePublisher<whole_body_state_msgs::WholeBodyTrajectory>
      pub_;
  pinocchio::Model model_;
  pinocchio::Data data_;
  std::string odom_frame_;
  Eigen::VectorXd a_null_;
};

} // namespace crocoddyl_msgs

#endif // CROCODDYL_MSG_WHOLE_BODY_TRAJECTORY_PUBLISHER_H_
