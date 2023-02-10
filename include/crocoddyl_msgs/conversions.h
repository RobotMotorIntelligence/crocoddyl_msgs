///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2020-2023, Heriot-Watt University, University of Oxford
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef CONVERSIONS_H_
#define CONVERSIONS_H_

#include "crocoddyl_msgs/Control.h"
#include "crocoddyl_msgs/FeedbackGain.h"
#include "crocoddyl_msgs/State.h"
#include <Eigen/Core>

namespace crocoddyl_msgs {

enum ControlType { EFFORT = 0, ACCELERATION_CONTACTFORCE };

enum ControlParametrization { POLYZERO = 0, POLYONE, POLYTWO };

/**
 * @brief Conversion of Eigen to message for a given
 * crocoddyl_msgs::FeedbackGain message reference
 *
 * @param[out] msg  ROS message that contains the feedback gain
 * @param[in] K     Feedback gain (size nu * nx)
 */
static inline void toMsg(crocoddyl_msgs::FeedbackGain &msg,
                         const Eigen::Ref<const Eigen::VectorXd> &K) {
  msg.nu = static_cast<uint32_t>(K.rows());
  msg.nx = static_cast<uint32_t>(K.cols());
  msg.data.resize(msg.nx * msg.nu);
  for (uint32_t i = 0; i < msg.nu; ++i) {
    for (uint32_t j = 0; j < msg.nx; ++j) {
      msg.data[i * msg.nx + j] = K(i, j);
    }
  }
}

/**
 * @brief Conversion of Eigen to message for a given crocoddyl_msgs::State
 * message reference
 *
 * @param[out] msg  ROS message that contains the state
 * @param[in] x     State at the beginning of the interval
 * @param[in] dx    State's rate of change during the interval
 */
static inline void toMsg(crocoddyl_msgs::State &msg,
                         const Eigen::Ref<const Eigen::VectorXd> &x,
                         const Eigen::Ref<const Eigen::VectorXd> &dx) {
  msg.x.resize(x.size());
  msg.dx.resize(dx.size());
  for (uint32_t i = 0; i < x.size(); ++i) {
    msg.x[i] = x(i);
  }
  for (uint32_t i = 0; i < dx.size(); ++i) {
    msg.dx[i] = dx(i);
  }
}

/**
 * @brief Conversion of Eigen to message for a given crocoddyl_msgs::Control
 * message reference
 *
 * @param[out] msg             ROS message that contains the control
 * @param[in] u                Control parameters of the interval
 * @param[in] K                Feedback gain of the interval
 * @param[in] type             Control type
 * @param[in] parametrization  Control parametrization
 */
static inline void toMsg(crocoddyl_msgs::Control &msg,
                         const Eigen::Ref<const Eigen::VectorXd> &u,
                         const Eigen::Ref<const Eigen::MatrixXd> &K,
                         const ControlType type,
                         const ControlParametrization parametrization) {
  msg.u.resize(u.size());
  for (uint32_t i = 0; i < u.size(); ++i) {
    msg.u[i] = u(i);
  }
  toMsg(msg.gain, K);
  switch (type) {
  case ControlType::EFFORT:
    msg.input = 0;
    break;
  case ControlType::ACCELERATION_CONTACTFORCE:
    msg.input = 1;
    break;
  default:
    msg.input = 0;
    break;
  }
  switch (parametrization) {
  case ControlParametrization::POLYZERO:
    msg.parametrization = 0;
    break;
  case ControlParametrization::POLYONE:
    msg.parametrization = 1;
    break;
  case ControlParametrization::POLYTWO:
    msg.parametrization = 1;
    break;
  default:
    msg.parametrization = 0;
    break;
  }
}

/**
 * @brief Conversion of a feedback gain from a crocoddyl_msgs::FeedbackGain
 * message to Eigen
 *
 * @param[in] msg  ROS message that contains the feedback gain
 * @param[out] K   Feedback gain (size nu * nx)
 */
static inline void fromMsg(const crocoddyl_msgs::FeedbackGain &msg,
                           Eigen::Ref<Eigen::MatrixXd> K) {
  if (K.rows() != msg.nu || K.cols() != msg.nx) {
    throw std::invalid_argument("The dimensions of K need to be: (" +
                                std::to_string(msg.nu) + ", " +
                                std::to_string(msg.nx) + ").");
  }
  if (msg.data.size() != msg.nu * msg.nx) {
    throw std::invalid_argument("Message incorrect - size of data does not "
                                "match given dimensions (nu,nx)");
  }

  for (std::size_t i = 0; i < msg.nu; ++i) {
    for (std::size_t j = 0; j < msg.nx; ++j) {
      K(i, j) = msg.data[i * msg.nx + j];
    }
  }
}

/**
 * @brief Conversion of a state from a crocoddyl_msgs::State message to Eigen
 *
 * @param[in] msg  ROS message that contains the state
 * @param[out] x   State at the beginning of the interval
 * @param[out] dx  State's rate of change during the interval
 */
static inline void fromMsg(const crocoddyl_msgs::State &msg,
                           Eigen::Ref<Eigen::VectorXd> x,
                           Eigen::Ref<Eigen::VectorXd> dx) {
  if (x.size() != msg.x.size()) {
    throw std::invalid_argument(
        "The x dimension needs to be: " + std::to_string(msg.x.size()) + ".");
  }
  if (dx.size() != msg.dx.size()) {
    throw std::invalid_argument(
        "The dx dimension needs to be: " + std::to_string(msg.dx.size()) + ".");
  }
  for (std::size_t i = 0; i < msg.x.size(); ++i) {
    x(i) = msg.x[i];
  }
  for (std::size_t i = 0; i < msg.dx.size(); ++i) {
    dx(i) = msg.dx[i];
  }
}

/**
 * @brief Conversion of a control from a crocoddyl_msgs::Control message to
 * Eigen
 *
 * @param[in] msg               ROS message that contains the control
 * @param[out] u                Control parameters of the interval
 * @param[out] K                Feedback gain of the interval
 * @param[out] type             Control type
 * @param[out] parametrization  Control parametrization
 */
static inline void fromMsg(const crocoddyl_msgs::Control &msg,
                           Eigen::Ref<Eigen::VectorXd> u,
                           Eigen::Ref<Eigen::MatrixXd> K, ControlType &type,
                           ControlParametrization &parametrization) {
  if (u.size() != msg.u.size()) {
    throw std::invalid_argument("The dimension of u needs to be: " +
                                std::to_string(msg.u.size()) + ".");
  }
  for (std::size_t i = 0; i < msg.u.size(); ++i) {
    u(i) = msg.u[i];
  }
  fromMsg(msg.gain, K);
  type = static_cast<ControlType>(msg.input);
  parametrization = static_cast<ControlParametrization>(msg.parametrization);
}

} // namespace crocoddyl_msgs

#endif // CONVERSIONS_H_