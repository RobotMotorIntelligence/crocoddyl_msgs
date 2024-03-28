///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2020-2023, Heriot-Watt University, University of Oxford
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef CROCODDYL_MSG_MULTIBODY_INERTIAL_PARAMETERS_SUBSCRIBER_H_
#define CROCODDYL_MSG_MULTIBODY_INERTIAL_PARAMETERS_SUBSCRIBER_H_

#include "crocoddyl_msgs/MultibodyInertialParameters.h"

#include <mutex>
#include <ros/node_handle.h>

typedef crocoddyl_msgs::MultibodyInertialParameters MultibodyInertialParameters;
typedef const crocoddyl_msgs::MultibodyInertialParameters::ConstPtr
    &MultibodyInertialParametersSharedPtr;
namespace crocoddyl_msgs {


class MultibodyInertialParametersRosSubscriber {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Initialize the multi-body inertial parameters subscriber.
   *
   * @param[in] n_bodies  Number of bodies
   * @param[in] topic     Topic name
   */
  MultibodyInertialParametersRosSubscriber(
      const unsigned int n_bodies,
      const std::string &topic = "/robot/multibody_inertial_parameters"):
      n_bodies_(n_bodies), spinner_(2), has_new_msg_(false), is_processing_msg_(false), last_msg_time_(0.) {
    ros::NodeHandle n;
    sub_ = n.subscribe<MultibodyInertialParameters>(
        topic, 1, &MultibodyInertialParametersRosSubscriber::callback, this,
        ros::TransportHints().tcpNoDelay());
    parameters.resize(10);
    spinner_.start();
    ROS_INFO_STREAM("Subscribing MultibodyInertialParameters messages on " << topic);
  }

  ~MultibodyInertialParametersRosSubscriber() = default;

  /**
   * @brief Get the latest inertial parameters
   *
   * @return  A map from body names to inertial parameters.
   */
  
  std::map<std::string, Eigen::VectorXd>
  get_inertial_parameters() {
    // start processing the message
    is_processing_msg_ = true;
    std::lock_guard<std::mutex> guard(mutex_);

    for (std::size_t i = 0; i<n_bodies_; ++i){
      const std::string &name =  msg_.parameters[i].name;
      parameters[0] = msg_.parameters[i].inertia.m;
      parameters[1] = msg_.parameters[i].inertia.com.x;
      parameters[2] = msg_.parameters[i].inertia.com.y;
      parameters[3] = msg_.parameters[i].inertia.com.z;
      parameters[4] = msg_.parameters[i].inertia.ixx;
      parameters[5] = msg_.parameters[i].inertia.ixy;
      parameters[6] = msg_.parameters[i].inertia.iyy;
      parameters[7] = msg_.parameters[i].inertia.ixz;
      parameters[8] = msg_.parameters[i].inertia.iyz;
      parameters[9] = msg_.parameters[i].inertia.izz;
      parameters_map[msg_.parameters[i].name] = parameters;
    }
    // finish processing the message
    is_processing_msg_ = false;
    has_new_msg_ = false;
    return parameters_map;
  }

  /**
   * @brief Indicate whether we have received a new message
   */
  bool has_new_msg() const { return has_new_msg_; }

private:
  ros::NodeHandle node_;
  ros::AsyncSpinner spinner_;
  ros::Subscriber sub_; //!< ROS subscriber
  std::mutex mutex_;      ///< Mutex to prevent race condition on callback
  MultibodyInertialParameters msg_;    //!< ROS message

  bool has_new_msg_;       //!< Indcate when a new message has been received
  bool is_processing_msg_; //!< Indicate when we are processing the message
  double last_msg_time_;

  unsigned int n_bodies_;
  Eigen::VectorXd parameters;
  std::map<std::string, Eigen::VectorXd> parameters_map;

  void callback(MultibodyInertialParametersSharedPtr msg) {
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

#endif // CROCODDYL_MSG_MULTIBODY_INERTIAL_PARAMETERS_SUBSCRIBER_H_
