///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2023-2024, Heriot-Watt University
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef CROCODDYL_MSG_MULTIBODY_INERTIAL_PARAMETERS_PUBLISHER_H_
#define CROCODDYL_MSG_MULTIBODY_INERTIAL_PARAMETERS_PUBLISHER_H_

#include <realtime_tools/realtime_publisher.h>

#include <ros/node_handle.h>
#include "crocoddyl_msgs/MultibodyInertialParameters.h"

typedef crocoddyl_msgs::MultibodyInertialParameters MultibodyInertialParameters;

namespace crocoddyl_msgs {

class MultibodyInertialParametersRosPublisher {
public:
  /**
   * @brief Initialize the multi-body inertial parameters publisher.
   *
   * @param[in] n_bodies  Number of bodies
   * @param[in] topic     Topic name
   */
  MultibodyInertialParametersRosPublisher(
      const std::string &topic = "/crocoddyl/inertial_parameters"){
    ros::NodeHandle n;
    pub_.init(n, topic, 1);
  }

  ~MultibodyInertialParametersRosPublisher() = default;

  /**
   * @brief Publish a multi-body inertial parameters ROS message.
   *
   * @param parameters[in]    multibody inertial parameters
   */
  void
  publish(const std::map<std::string, const Eigen::Ref<const Eigen::VectorXd> > &parameters
          ) {
    const std::size_t n_bodies = parameters.size();
    pub_.msg_.parameters.resize(n_bodies);

    if (pub_.trylock()) {
      pub_.msg_.header.stamp = ros::Time::now();
      unsigned int i = 0;
      for (const auto& [body_name, psi] : parameters){
        if (psi.size() != 10)
          throw std::invalid_argument(
            "Dimension of psi for body " + body_name + " is not 10");
        pub_.msg_.parameters[i].name = body_name;
        pub_.msg_.parameters[i].inertia.m = psi[0];      
        pub_.msg_.parameters[i].inertia.com.x = psi[1]; 
        pub_.msg_.parameters[i].inertia.com.y = psi[2];
        pub_.msg_.parameters[i].inertia.com.z = psi[3];
        pub_.msg_.parameters[i].inertia.ixx = psi[4];
        pub_.msg_.parameters[i].inertia.ixy = psi[5];
        pub_.msg_.parameters[i].inertia.iyy = psi[6];
        pub_.msg_.parameters[i].inertia.ixz = psi[7];
        pub_.msg_.parameters[i].inertia.iyz = psi[8];
        pub_.msg_.parameters[i].inertia.izz = psi[9];
        i++;
      }
      pub_.unlockAndPublish();
    }
  }

private:
  realtime_tools::RealtimePublisher<MultibodyInertialParameters> pub_;
};

} // namespace crocoddyl_msgs

#endif // CROCODDYL_MSG_MULTIBODY_INERTIAL_PARAMETERS_PUBLISHER_H_
