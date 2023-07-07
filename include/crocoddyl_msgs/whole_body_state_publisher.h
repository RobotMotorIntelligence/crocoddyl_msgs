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
   * @brief Initialize the whole-body state publisher
   *
   * @param[in] model  Pinocchio model
   * @param[in] topic  Topic name
   * @param[in] frame  Odometry frame
   */
#ifdef ROS2
  WholeBodyStateRosPublisher(pinocchio::Model &model, const std::string &topic = "/crocoddyl/whole_body_state",
                             const std::string &frame = "odom")
      : node_("whole_body_state_publisher"),
        pub_(node_.create_publisher<WholeBodyState>(topic, 1)),
        model_(model),
        data_(model),
        odom_frame_(frame),
        a_(Eigen::VectorXd::Zero(model.nv)) {
#else
  WholeBodyStateRosPublisher(pinocchio::Model &model, const std::string &topic = "/crocoddyl/whole_body_state",
                             const std::string &frame = "odom")
      : model_(model), data_(model), odom_frame_(frame), a_(Eigen::VectorXd::Zero(model.nv)) {
    ros::NodeHandle n;
    pub_.init(n, topic, 1);
#endif
    pub_.msg_.header.frame_id = frame;
    std::cout << "Publish WholeBodyState messages on " << topic << "(frame: " << frame << ")" << std::endl;
  }
  ~WholeBodyStateRosPublisher() = default;

  /**
   * @brief Publish a whole-body state ROS message
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
  void publish(const double t, const Eigen::Ref<const Eigen::VectorXd> &q, const Eigen::Ref<const Eigen::VectorXd> &v,
               const Eigen::Ref<const Eigen::VectorXd> &tau, const std::map<std::string, pinocchio::SE3> &p,
               const std::map<std::string, pinocchio::Motion> &pd,
               const std::map<std::string, std::tuple<pinocchio::Force, ContactType, ContactStatus>> &f,
               const std::map<std::string, std::pair<Eigen::Vector3d, double>> &s) {
    if (pub_.trylock()) {
      pub_.msg_.header.frame_id = odom_frame_;
      crocoddyl_msgs::toMsg(model_, data_, pub_.msg_, t, q, v, a_, tau, p, pd, f, s);
      pub_.unlockAndPublish();
    }
  }

 private:
#ifdef ROS2
  rclcpp::Node node_;
#endif
  realtime_tools::RealtimePublisher<WholeBodyState> pub_;
  pinocchio::Model model_;
  pinocchio::Data data_;
  std::string odom_frame_;
  Eigen::VectorXd a_;
};

}  // namespace crocoddyl_msgs

#endif  // CROCODDYL_MSG_WHOLE_BODY_STATE_PUBLISHER_H_
