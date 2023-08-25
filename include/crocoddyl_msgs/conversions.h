///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2020-2023, Heriot-Watt University, University of Oxford
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef CONVERSIONS_H_
#define CONVERSIONS_H_

#include <pinocchio/fwd.hpp>

#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/container/aligned-vector.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/spatial/force.hpp>
#include <pinocchio/spatial/motion.hpp>

#include <pinocchio/bindings/python/pybind11.hpp>
#define SCALAR double
#define OPTIONS 0
#define JOINT_MODEL_COLLECTION ::pinocchio::JointCollectionDefaultTpl
#include <pinocchio/bindings/python/pybind11-all.hpp>

#ifdef ROS2
#include "crocoddyl_msgs/msg/control.hpp"
#include "crocoddyl_msgs/msg/feedback_gain.hpp"
#include "crocoddyl_msgs/msg/state.hpp"
#include "crocoddyl_msgs/msg/time_interval.hpp"
#include <whole_body_state_msgs/msg/whole_body_state.hpp>
#include <whole_body_state_msgs/msg/whole_body_trajectory.hpp>
#else
#include "crocoddyl_msgs/Control.h"
#include "crocoddyl_msgs/FeedbackGain.h"
#include "crocoddyl_msgs/State.h"
#include "crocoddyl_msgs/TimeInterval.h"
#include <whole_body_state_msgs/WholeBodyState.h>
#include <whole_body_state_msgs/WholeBodyTrajectory.h>
#endif

namespace crocoddyl_msgs {

enum ControlType { EFFORT = 0, ACCELERATION_CONTACTFORCE };

enum ControlParametrization { POLYZERO = 0, POLYONE, POLYTWO };

enum ContactType { LOCOMOTION = 0, MANIPULATION };

enum ContactStatus { UNKNOWN = 0, SEPARATION, STICKING, SLIPPING };

#ifdef ROS2
typedef crocoddyl_msgs::msg::TimeInterval TimeInterval;
typedef crocoddyl_msgs::msg::State State;
typedef crocoddyl_msgs::msg::Control Control;
typedef crocoddyl_msgs::msg::FeedbackGain FeedbackGain;
typedef whole_body_state_msgs::msg::WholeBodyState WholeBodyState;
typedef whole_body_state_msgs::msg::WholeBodyTrajectory WholeBodyTrajectory;
typedef whole_body_state_msgs::msg::ContactState ContactState;
#else
typedef crocoddyl_msgs::TimeInterval TimeInterval;
typedef crocoddyl_msgs::State State;
typedef crocoddyl_msgs::Control Control;
typedef crocoddyl_msgs::FeedbackGain FeedbackGain;
typedef whole_body_state_msgs::WholeBodyState WholeBodyState;
typedef whole_body_state_msgs::WholeBodyTrajectory WholeBodyTrajectory;
typedef whole_body_state_msgs::ContactState ContactState;
#endif

/**
 * @brief Return the root joint id
 *
 * @param return  Root joint Id
 */
template <int Options, template <typename, int> class JointCollectionTpl>
static inline std::size_t get_root_joint_id(
    const pinocchio::ModelTpl<double, Options, JointCollectionTpl> &model) {
  return model.existJointName("root_joint")
             ? model.getJointId("root_joint")
             : (model.existJointName("freeflyer_joint")
                    ? model.getJointId("freeflyer_joint")
                    : 0);
}

/**
 * @brief Conversion of Eigen to message for a given
 * crocoddyl_msgs::FeedbackGain message reference
 *
 * @param[out] msg  ROS message that contains the feedback gain
 * @param[in] K     Feedback gain (size nu * nx)
 */
static inline void toMsg(FeedbackGain &msg,
                         const Eigen::Ref<const Eigen::MatrixXd> &K) {
  msg.nu = static_cast<uint32_t>(K.rows());
  msg.nx = static_cast<uint32_t>(K.cols());
  msg.data.resize(msg.nx * msg.nu);
  for (uint32_t i = 0; i < msg.nu; ++i) {
    for (uint32_t j = 0; j < msg.nx; ++j) {
      msg.data[i * msg.nx + j] = K(i, j); // store in row-major order
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
static inline void toMsg(State &msg, const Eigen::Ref<const Eigen::VectorXd> &x,
                         const Eigen::Ref<const Eigen::VectorXd> &dx) {
  msg.x.resize(x.size());
  msg.dx.resize(dx.size());
  for (int i = 0; i < x.size(); ++i) {
    msg.x[i] = x(i);
  }
  for (int i = 0; i < dx.size(); ++i) {
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
static inline void toMsg(Control &msg,
                         const Eigen::Ref<const Eigen::VectorXd> &u,
                         const Eigen::Ref<const Eigen::MatrixXd> &K,
                         const ControlType type,
                         const ControlParametrization parametrization) {
  msg.u.resize(u.size());
  for (int i = 0; i < u.size(); ++i) {
    msg.u[i] = u(i);
  }
  toMsg(msg.gain, K);
  switch (type) {
  case ControlType::EFFORT:
    msg.input = crocoddyl_msgs::Control::EFFORT;
    break;
  case ControlType::ACCELERATION_CONTACTFORCE:
    msg.input = crocoddyl_msgs::Control::ACCELERATION_CONTACTFORCE;
    break;
  }
  switch (parametrization) {
  case ControlParametrization::POLYZERO:
    msg.parametrization = crocoddyl_msgs::Control::POLYZERO;
    break;
  case ControlParametrization::POLYONE:
    msg.parametrization = crocoddyl_msgs::Control::POLYONE;
    break;
  case ControlParametrization::POLYTWO:
    msg.parametrization = crocoddyl_msgs::Control::POLYTWO;
    break;
  }
}

/**
 * @brief Conversion from vectors to `whole_body_state_msgs::WholeBodyState`
 *
 * @param model[in]  Pinocchio model
 * @param data[out]  Pinocchio data
 * @param msg[out]   ROS message that contains the whole-body state
 * @param t[in]      Time in secs
 * @param q[in]      Configuration vector (dimension: model.nq)
 * @param v[in]      Generalized velocity (dimension: model.nv)
 * @param a[in]      Generalized acceleration (dimension: model.nv)
 * @param tau[in]    Joint effort
 * @param p[in]      Contact position
 * @param pd[in]     Contact velocity
 * @param f[in]      Contact force, type and status
 * @param s[in]      Contact surface and friction coefficient
 */
template <int Options, template <typename, int> class JointCollectionTpl>
static inline void toMsg(
    const pinocchio::ModelTpl<double, Options, JointCollectionTpl> &model,
    pinocchio::DataTpl<double, Options, JointCollectionTpl> &data,
    WholeBodyState &msg, const double t,
    const Eigen::Ref<const Eigen::VectorXd> &q,
    const Eigen::Ref<const Eigen::VectorXd> &v,
    const Eigen::Ref<const Eigen::VectorXd> &a,
    const Eigen::Ref<const Eigen::VectorXd> &tau,
    const std::map<std::string, pinocchio::SE3> &p,
    const std::map<std::string, pinocchio::Motion> &pd,
    const std::map<std::string,
                   std::tuple<pinocchio::Force, ContactType, ContactStatus>> &f,
    const std::map<std::string, std::pair<Eigen::Vector3d, double>> &s) {
  if (q.size() != model.nq) {
    throw std::invalid_argument("Expected q to be " + std::to_string(model.nq) +
                                " but received " + std::to_string(q.size()));
  }
  if (v.size() != model.nv) {
    throw std::invalid_argument("Expected v to be " + std::to_string(model.nv) +
                                " but received " + std::to_string(v.size()));
  }
  if (a.size() != model.nv) {
    throw std::invalid_argument("Expected a to be " + std::to_string(model.nv) +
                                " but received " + std::to_string(a.size()));
  }
  const std::size_t root_joint_id = get_root_joint_id(model);
  const std::size_t nv_root = model.joints[root_joint_id].nv();
  const std::size_t njoints = model.nv - nv_root;
  if (tau.size() != static_cast<int>(njoints) && tau.size() != 0) {
    throw std::invalid_argument("Expected tau to be 0 or " +
                                std::to_string(njoints) + " but received " +
                                std::to_string(tau.size()));
  }
  if (p.size() != pd.size()) {
    throw std::invalid_argument(
        "Dimension of contact pose and velocity does not match.");
  }
  if (p.size() != f.size()) {
    throw std::invalid_argument(
        "Dimension of contact pose and force does not match.");
  }
  if (p.size() != s.size()) {
    throw std::invalid_argument(
        "Dimension of contact pose and surface does not match.");
  }
  // Filling the time information
  msg.time = t;
#ifdef ROS2
  msg.header.stamp = rclcpp::Time(static_cast<uint64_t>(t * 1e9));
#else
  msg.header.stamp = ros::Time(t);
#endif
  // Filling the centroidal state
  pinocchio::centerOfMass(model, data, q, v, a);
  // Center of mass
  msg.centroidal.com_position.x = data.com[0].x();
  msg.centroidal.com_position.y = data.com[0].y();
  msg.centroidal.com_position.z = data.com[0].z();
  // Velocity of the CoM expressed in the global frame.
  msg.centroidal.com_velocity.x = data.vcom[0].x();
  msg.centroidal.com_velocity.y = data.vcom[0].y();
  msg.centroidal.com_velocity.z = data.vcom[0].z();
  // Base
  if (nv_root == 6) { // TODO(cmastalli): handle other root joints
    msg.centroidal.base_orientation.x = q(3);
    msg.centroidal.base_orientation.y = q(4);
    msg.centroidal.base_orientation.z = q(5);
    msg.centroidal.base_orientation.w = q(6);
    msg.centroidal.base_angular_velocity.x = v(3);
    msg.centroidal.base_angular_velocity.y = v(4);
    msg.centroidal.base_angular_velocity.z = v(5);
  } else if (nv_root != 0) {
    std::cerr
        << "Warning: toMsg conversion does not yet support root joints "
           "different to a floating base. We cannot publish base information."
        << std::endl;
  }
  // Momenta
  const pinocchio::Force &momenta =
      pinocchio::computeCentroidalMomentum(model, data);
  msg.centroidal.momenta.linear.x = momenta.linear().x();
  msg.centroidal.momenta.linear.y = momenta.linear().y();
  msg.centroidal.momenta.linear.z = momenta.linear().z();
  msg.centroidal.momenta.angular.x = momenta.angular().x();
  msg.centroidal.momenta.angular.y = momenta.angular().y();
  msg.centroidal.momenta.angular.z = momenta.angular().z();
  const pinocchio::Force &momenta_rate =
      pinocchio::computeCentroidalMomentumTimeVariation(model, data);
  msg.centroidal.momenta_rate.linear.x = momenta_rate.linear().x();
  msg.centroidal.momenta_rate.linear.y = momenta_rate.linear().y();
  msg.centroidal.momenta_rate.linear.z = momenta_rate.linear().z();
  msg.centroidal.momenta_rate.angular.x = momenta_rate.angular().x();
  msg.centroidal.momenta_rate.angular.y = momenta_rate.angular().y();
  msg.centroidal.momenta_rate.angular.z = momenta_rate.angular().z();
  // Filling the joint state
  msg.joints.resize(njoints);
  for (std::size_t j = 0; j < njoints; ++j) {
    msg.joints[j].name = model.names[2 + j];
    msg.joints[j].position = q(model.joints[1].nq() + j);
    msg.joints[j].velocity = v(model.joints[1].nv() + j);
    msg.joints[j].acceleration = a(model.joints[1].nv() + j);
    msg.joints[j].effort = tau(j);
  }
  // Contacts
  msg.contacts.resize(p.size());
  std::size_t i = 0;
  for (const auto &p_item : p) {
    const std::string &name = p_item.first;
    msg.contacts[i].name = name;
    pinocchio::FrameIndex frame_id = model.getFrameId(name);
    if (static_cast<int>(frame_id) > model.nframes) {
      throw std::runtime_error("Frame '" + name + "' not found.");
    }
    std::map<std::string, pinocchio::Motion>::const_iterator pd_it =
        pd.find(name);
    if (pd_it == pd.end()) {
      throw std::runtime_error("Frame '" + name + "' not found in pd.");
    }
    std::map<std::string, std::tuple<pinocchio::Force, ContactType,
                                     ContactStatus>>::const_iterator f_it =
        f.find(name);
    if (f_it == f.end()) {
      throw std::runtime_error("Frame '" + name + "' not found in f.");
    }
    std::map<std::string, std::pair<Eigen::Vector3d, double>>::const_iterator
        s_it = s.find(name);
    if (s_it == s.end()) {
      throw std::runtime_error("Frame '" + name + "' not found in s.");
    }
    ++i;
  }
  i = 0;
  for (const auto &p_item : p) {
    const pinocchio::SE3 &pose = p_item.second;
    pinocchio::SE3::Quaternion quaternion(pose.rotation());
    msg.contacts[i].pose.position.x = pose.translation().x();
    msg.contacts[i].pose.position.y = pose.translation().y();
    msg.contacts[i].pose.position.z = pose.translation().z();
    msg.contacts[i].pose.orientation.x = quaternion.x();
    msg.contacts[i].pose.orientation.y = quaternion.y();
    msg.contacts[i].pose.orientation.z = quaternion.z();
    msg.contacts[i].pose.orientation.w = quaternion.w();
    ++i;
  }
  i = 0;
  for (const auto &pd_item : pd) {
    const pinocchio::Motion &vel = pd_item.second;
    msg.contacts[i].velocity.linear.x = vel.linear().x();
    msg.contacts[i].velocity.linear.y = vel.linear().y();
    msg.contacts[i].velocity.linear.z = vel.linear().z();
    msg.contacts[i].velocity.angular.x = vel.angular().x();
    msg.contacts[i].velocity.angular.y = vel.angular().y();
    msg.contacts[i].velocity.angular.z = vel.angular().z();
    ++i;
  }
  i = 0;
  for (const auto &f_item : f) {
    const std::tuple<pinocchio::Force, ContactType, ContactStatus> &force =
        f_item.second;
    const pinocchio::Force &wrench = std::get<0>(force);
    const ContactType type = std::get<1>(force);
    switch (type) {
    case ContactType::LOCOMOTION:
      msg.contacts[i].type = ContactState::LOCOMOTION;
      break;
    case ContactType::MANIPULATION:
      msg.contacts[i].type = ContactState::MANIPULATION;
      break;
    }
    const ContactStatus status = std::get<2>(force);
    switch (status) {
    case ContactStatus::UNKNOWN:
      msg.contacts[i].status = ContactState::UNKNOWN;
      break;
    case ContactStatus::SEPARATION:
      msg.contacts[i].status = ContactState::INACTIVE;
      break;
    case ContactStatus::STICKING:
      msg.contacts[i].status = ContactState::ACTIVE;
      break;
    case ContactStatus::SLIPPING:
      msg.contacts[i].status = ContactState::SLIPPING;
      break;
    }
    msg.contacts[i].wrench.force.x = wrench.linear().x();
    msg.contacts[i].wrench.force.y = wrench.linear().y();
    msg.contacts[i].wrench.force.z = wrench.linear().z();
    msg.contacts[i].wrench.torque.x = wrench.angular().x();
    msg.contacts[i].wrench.torque.y = wrench.angular().y();
    msg.contacts[i].wrench.torque.z = wrench.angular().z();
    ++i;
  }
  i = 0;
  for (const auto &s_item : s) {
    const std::pair<Eigen::Vector3d, double> &surf = s_item.second;
    const Eigen::Vector3d &norm = std::get<0>(surf);
    msg.contacts[i].surface_normal.x = norm.x();
    msg.contacts[i].surface_normal.y = norm.y();
    msg.contacts[i].surface_normal.z = norm.z();
    msg.contacts[i].friction_coefficient = std::get<1>(surf);
    ++i;
  }
}

/**
 * @brief Conversion of a feedback gain from a crocoddyl_msgs::FeedbackGain
 * message to Eigen
 *
 * @param[in] msg  ROS message that contains the feedback gain
 * @param[out] K   Feedback gain (size nu * nx)
 */
static inline void fromMsg(const FeedbackGain &msg,
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
static inline void fromMsg(const State &msg, Eigen::Ref<Eigen::VectorXd> x,
                           Eigen::Ref<Eigen::VectorXd> dx) {
  if (static_cast<std::size_t>(x.size()) != msg.x.size()) {
    throw std::invalid_argument("Expected x to be " +
                                std::to_string(msg.x.size()) +
                                " but received " + std::to_string(x.size()));
  }
  if (static_cast<std::size_t>(dx.size()) != msg.dx.size()) {
    throw std::invalid_argument("Expected dx to be " +
                                std::to_string(msg.dx.size()) +
                                " but received " + std::to_string(dx.size()));
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
static inline void fromMsg(const Control &msg, Eigen::Ref<Eigen::VectorXd> u,
                           Eigen::Ref<Eigen::MatrixXd> K, ControlType &type,
                           ControlParametrization &parametrization) {
  if (static_cast<std::size_t>(u.size()) != msg.u.size()) {
    throw std::invalid_argument("Expected u to be " +
                                std::to_string(msg.u.size()) +
                                " but received " + std::to_string(u.size()));
  }
  for (std::size_t i = 0; i < msg.u.size(); ++i) {
    u(i) = msg.u[i];
  }
  fromMsg(msg.gain, K);
  type = static_cast<ControlType>(msg.input);
  parametrization = static_cast<ControlParametrization>(msg.parametrization);
}

/**
 * @brief Conversion from whole_body_state_msgs::WholeBodyState to deserialized
 * quantities
 *
 * @param model[in]  Pinocchio model
 * @param data[out]  Pinocchio data
 * @param msg[in]    ROS message that contains the whole-body state
 * @param t[out]     Time in secs
 * @param q[out]     Configuration vector (dimension: model.nq)
 * @param v[out]     Generalized velocity (dimension: model.nv)
 * @param a[out]     Generalized acceleratio (dimension: model.nv)
 * @param tau[out]   Joint effort
 * @param p[out]     Contact position
 * @param pd[out]    Contact velocity
 * @param f[out]     Contact force, type and status
 * @param s[out]     Contact surface and friction coefficient
 */
template <int Options, template <typename, int> class JointCollectionTpl>
static inline void
fromMsg(const pinocchio::ModelTpl<double, Options, JointCollectionTpl> &model,
        pinocchio::DataTpl<double, Options, JointCollectionTpl> &data,
        const WholeBodyState &msg, double &t, Eigen::Ref<Eigen::VectorXd> q,
        Eigen::Ref<Eigen::VectorXd> v, Eigen::Ref<Eigen::VectorXd> a,
        Eigen::Ref<Eigen::VectorXd> tau,
        std::map<std::string, pinocchio::SE3> &p,
        std::map<std::string, pinocchio::Motion> &pd,
        std::map<std::string,
                 std::tuple<pinocchio::Force, ContactType, ContactStatus>> &f,
        std::map<std::string, std::pair<Eigen::Vector3d, double>> &s) {
  if (q.size() != model.nq) {
    throw std::invalid_argument("Expected q to be " + std::to_string(model.nq) +
                                " but received " + std::to_string(q.size()));
  }
  if (v.size() != model.nv) {
    throw std::invalid_argument("Expected v to be " + std::to_string(model.nv) +
                                " but received " + std::to_string(v.size()));
  }
  if (a.size() != model.nv) {
    throw std::invalid_argument("Expected a to be " + std::to_string(model.nv) +
                                " but received " + std::to_string(v.size()));
  }
  const std::size_t root_joint_id = get_root_joint_id(model);
  const std::size_t nv_root = model.joints[root_joint_id].nv();
  const std::size_t njoints = model.nv - nv_root;
  if (tau.size() != static_cast<int>(njoints)) {
    throw std::invalid_argument("Expected tau to be " +
                                std::to_string(njoints) + " but received " +
                                std::to_string(tau.size()));
  }
  if (msg.joints.size() != static_cast<std::size_t>(njoints)) {
    throw std::invalid_argument("Message incorrect - msg.joints size is " +
                                std::to_string(msg.joints.size()) +
                                " but expected to be " +
                                std::to_string(njoints));
  }
  t = msg.time;
  // Retrieve the joint state
  for (std::size_t j = 0; j < njoints; ++j) {
    auto joint_id = model.getJointId(msg.joints[j].name);
    auto q_idx = model.idx_qs[joint_id];
    auto v_idx = model.idx_vs[joint_id];
    q(q_idx) = msg.joints[j].position;
    v(v_idx) = msg.joints[j].velocity;
    a(v_idx) = msg.joints[j].acceleration;
    tau(v_idx - nv_root) = msg.joints[j].effort;
  }

  // Retrieve the base state
  if (nv_root == 6) {
    q.head<3>().setZero();
    v.head<3>().setZero();
    q(3) = msg.centroidal.base_orientation.x;
    q(4) = msg.centroidal.base_orientation.y;
    q(5) = msg.centroidal.base_orientation.z;
    q(6) = msg.centroidal.base_orientation.w;
    v(3) = msg.centroidal.base_angular_velocity.x;
    v(4) = msg.centroidal.base_angular_velocity.y;
    v(5) = msg.centroidal.base_angular_velocity.z;

    pinocchio::normalize(model, q);
    pinocchio::centerOfMass(model, data, q, v, a);
    q(0) = msg.centroidal.com_position.x - data.com[0](0);
    q(1) = msg.centroidal.com_position.y - data.com[0](1);
    q(2) = msg.centroidal.com_position.z - data.com[0](2);
    v(0) = msg.centroidal.com_velocity.x - data.vcom[0](0);
    v(1) = msg.centroidal.com_velocity.y - data.vcom[0](1);
    v(2) = msg.centroidal.com_velocity.z - data.vcom[0](2);
    v.head<3>() = Eigen::Quaterniond(q(6), q(3), q(4),
                                     q(5))
                      .toRotationMatrix()
                      .transpose() *
                  v.head<3>(); // local frame
  } else if (nv_root != 0) {
    std::cerr
        << "Warning: fromMsg conversion does not yet support root joints "
           "different to a floating base. We cannot publish base information."
        << std::endl;
  }

  // Retrieve the contact information
  for (const auto &contact : msg.contacts) {
    // Contact pose
    p[contact.name] = pinocchio::SE3(
        Eigen::Quaterniond(
            contact.pose.orientation.w, contact.pose.orientation.x,
            contact.pose.orientation.y, contact.pose.orientation.z),
        Eigen::Vector3d(contact.pose.position.x, contact.pose.position.y,
                        contact.pose.position.z));
    // Contact velocity
    pd[contact.name] = pinocchio::Motion(
        Eigen::Vector3d(contact.velocity.linear.x, contact.velocity.linear.y,
                        contact.velocity.linear.z),
        Eigen::Vector3d(contact.velocity.angular.x, contact.velocity.angular.y,
                        contact.velocity.angular.z));
    // Contact wrench
    ContactType type;
    switch (contact.type) {
    case ContactState::LOCOMOTION:
      type = ContactType::LOCOMOTION;
      break;
    case ContactState::MANIPULATION:
      type = ContactType::MANIPULATION;
      break;
    }
    ContactStatus status;
    switch (contact.status) {
    case ContactState::UNKNOWN:
      status = ContactStatus::UNKNOWN;
      break;
    case ContactState::INACTIVE:
      status = ContactStatus::SEPARATION;
      break;
    case ContactState::ACTIVE:
      status = ContactStatus::STICKING;
      break;
    case ContactState::SLIPPING:
      status = ContactStatus::SLIPPING;
      break;
    }
    f[contact.name] = {
        pinocchio::Force(
            Eigen::Vector3d(contact.wrench.force.x, contact.wrench.force.y,
                            contact.wrench.force.z),
            Eigen::Vector3d(contact.wrench.torque.x, contact.wrench.torque.y,
                            contact.wrench.torque.z)),
        type, status};
    // Surface normal and friction coefficient
    s[contact.name] = {Eigen::Vector3d(contact.surface_normal.x,
                                       contact.surface_normal.y,
                                       contact.surface_normal.z),
                       contact.friction_coefficient};
  }
}

/**
 * @brief Conversion from reduced position, velocity and effort to a full one
 *
 * @param model[in]  Pinocchio model
 * @param reduced_model[in]  Reduced Pinocchio model
 * @param q_out[out]  Configuration vector (dimension: model.nq)
 * @param v_out[out]  Generalized velocity (dimension: model.nv)
 * @param tau_out[out]  Joint effort (dimension: model.nv - nv_root)
 * @param q[in]  Reduced configuration vector (dimension: reduced_model.nq)
 * @param v[in]  Reduced generalized velocity (dimension: reduced_model.nv)
 * @param tau[in]  Reduced joint effort (dimension: reduced_model.nv - nv_root)
 * @param qref[in]  Reference configuration used in the reduced model
 * @param locked_joint_ids[in]  Ids of the locked joints
 */
template <int Options, template <typename, int> class JointCollectionTpl>
static inline void fromReduced(
    const pinocchio::ModelTpl<double, Options, JointCollectionTpl> &model,
    const pinocchio::ModelTpl<double, Options, JointCollectionTpl>
        &reduced_model,
    Eigen::Ref<Eigen::VectorXd> q_out, Eigen::Ref<Eigen::VectorXd> v_out,
    Eigen::Ref<Eigen::VectorXd> tau_out,
    const Eigen::Ref<const Eigen::VectorXd> &q_in,
    const Eigen::Ref<const Eigen::VectorXd> &v_in,
    const Eigen::Ref<const Eigen::VectorXd> &tau_in,
    const Eigen::Ref<const Eigen::VectorXd> &qref,
    const std::vector<pinocchio::JointIndex> &locked_joint_ids) {
  const std::size_t root_joint_id = get_root_joint_id(model);
  const std::size_t nq_root = model.joints[root_joint_id].nq();
  const std::size_t nv_root = model.joints[root_joint_id].nv();
  if (q_out.size() != model.nq) {
    throw std::invalid_argument("Expected q_out to be " +
                                std::to_string(model.nq) + " but received " +
                                std::to_string(q_out.size()));
  }
  if (q_in.size() != reduced_model.nq) {
    throw std::invalid_argument("Expected q_in to be " +
                                std::to_string(reduced_model.nq) +
                                " but received " + std::to_string(q_in.size()));
  }
  if (v_out.size() != model.nv) {
    throw std::invalid_argument("Expected v_out to be " +
                                std::to_string(model.nv) + " but received " +
                                std::to_string(v_out.size()));
  }
  if (v_in.size() != reduced_model.nv) {
    throw std::invalid_argument("Expected v_in to be " +
                                std::to_string(reduced_model.nv) +
                                " but received " + std::to_string(v_in.size()));
  }
  if (static_cast<std::size_t>(tau_out.size()) != model.nv - nv_root) {
    throw std::invalid_argument(
        "Expected tau_out to be " + std::to_string(model.nv - nv_root) +
        " but received " + std::to_string(tau_out.size()));
  }
  if (static_cast<std::size_t>(tau_in.size()) != reduced_model.nv - nv_root) {
    throw std::invalid_argument(
        "Expected tau_in to be " + std::to_string(reduced_model.nv - nv_root) +
        " but received " + std::to_string(tau_in.size()));
  }

  typedef pinocchio::ModelTpl<double, Options, JointCollectionTpl> Model;
  typedef typename Model::JointModel JointModel;

  q_out.head(nq_root) = q_in.head(nq_root);
  v_out.head(nv_root) = v_in.head(nv_root);
  for (std::size_t j = root_joint_id;
       j < static_cast<std::size_t>(model.njoints); ++j) {
    const std::string &name = model.names[j];
    JointModel joint = model.joints[model.getJointId(name)];
    JointModel reduced_joint = model.joints[reduced_model.getJointId(name)];
    q_out(joint.idx_q()) = q_in(reduced_joint.idx_q());
    v_out(joint.idx_v()) = v_in(reduced_joint.idx_v());
    tau_out(joint.idx_v() - nv_root) = tau_in(reduced_joint.idx_v() - nv_root);
  }
  for (pinocchio::JointIndex joint_id : locked_joint_ids) {
    JointModel joint = model.joints[joint_id];
    q_out(joint.idx_q()) = qref(joint.idx_q());
    v_out(joint.idx_v()) = 0.;
    tau_out(joint.idx_v() - nv_root) = 0.;
  }
}

/**
 * @brief Conversion to reduced position, velocity and effort from a full one
 *
 * @param model[in]  Pinocchio model
 * @param reduced_model[in]  Reduced Pinocchio model
 * @param q_out[out]  Reduced configuration vector (dimension: reduced_model.nq)
 * @param v_out[out]  Reduced generalized velocity (dimension: reduced_model.nv)
 * @param tau_out[out]  Reduced joint effort (dimension: reduced_model.nv -
 * nv_root)
 * @param q[in]  Configuration vector (dimension: model.nq)
 * @param v[in]  Generalized velocity (dimension: model.nv)
 * @param tau[in]  Joint effort (dimension: model.nv - nv_root)
 */
template <int Options, template <typename, int> class JointCollectionTpl>
static inline void
toReduced(const pinocchio::ModelTpl<double, Options, JointCollectionTpl> &model,
          const pinocchio::ModelTpl<double, Options, JointCollectionTpl>
              &reduced_model,
          Eigen::Ref<Eigen::VectorXd> q_out, Eigen::Ref<Eigen::VectorXd> v_out,
          Eigen::Ref<Eigen::VectorXd> tau_out,
          const Eigen::Ref<const Eigen::VectorXd> &q_in,
          const Eigen::Ref<const Eigen::VectorXd> &v_in,
          const Eigen::Ref<const Eigen::VectorXd> &tau_in) {
  const std::size_t root_joint_id = get_root_joint_id(model);
  const std::size_t nq_root = model.joints[root_joint_id].nq();
  const std::size_t nv_root = model.joints[root_joint_id].nv();
  if (q_out.size() != reduced_model.nq) {
    throw std::invalid_argument(
        "Expected q_out to be " + std::to_string(reduced_model.nq) +
        " but received " + std::to_string(q_out.size()));
  }
  if (q_in.size() != model.nq) {
    throw std::invalid_argument("Expected q_in to be " +
                                std::to_string(model.nq) + " but received " +
                                std::to_string(q_in.size()));
  }
  if (v_out.size() != reduced_model.nv) {
    throw std::invalid_argument(
        "Expected v_out to be " + std::to_string(reduced_model.nv) +
        " but received " + std::to_string(v_out.size()));
  }
  if (v_in.size() != model.nv) {
    throw std::invalid_argument("Expected v_in to be " +
                                std::to_string(model.nv) + " but received " +
                                std::to_string(v_in.size()));
  }
  if (static_cast<std::size_t>(tau_out.size()) != reduced_model.nv - nv_root) {
    throw std::invalid_argument(
        "Expected tau_out to be " + std::to_string(reduced_model.nv - nv_root) +
        " but received " + std::to_string(tau_out.size()));
  }
  if (static_cast<std::size_t>(tau_in.size()) != model.nv - nv_root) {
    throw std::invalid_argument(
        "Expected tau_in to be " + std::to_string(model.nv - nv_root) +
        " but received " + std::to_string(tau_in.size()));
  }

  typedef pinocchio::ModelTpl<double, Options, JointCollectionTpl> Model;
  typedef typename Model::JointModel JointModel;

  q_out.head(nq_root) = q_in.head(nq_root);
  v_out.head(nv_root) = v_in.head(nv_root);
  for (std::size_t j = root_joint_id;
       j < static_cast<std::size_t>(model.njoints); ++j) {
    const std::string &name = model.names[j];
    JointModel joint = model.joints[model.getJointId(name)];
    JointModel reduced_joint = model.joints[reduced_model.getJointId(name)];
    q_out(reduced_joint.idx_q()) = q_in(joint.idx_q());
    v_out(reduced_joint.idx_v()) = v_in(joint.idx_v());
    tau_out(reduced_joint.idx_v() - nv_root) = tau_in(joint.idx_v() - nv_root);
  }
}

template <int Options, template <typename, int> class JointCollectionTpl>
static inline std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd>
fromReduced_return(
    const pinocchio::ModelTpl<double, Options, JointCollectionTpl> &model,
    const pinocchio::ModelTpl<double, Options, JointCollectionTpl>
        &reduced_model,
    const Eigen::Ref<const Eigen::VectorXd> &q_in,
    const Eigen::Ref<const Eigen::VectorXd> &v_in,
    const Eigen::Ref<const Eigen::VectorXd> &tau_in,
    const Eigen::Ref<const Eigen::VectorXd> &qref,
    const std::vector<pinocchio::JointIndex> &locked_joint_ids) {
  const std::size_t root_joint_id = get_root_joint_id(model);
  Eigen::VectorXd q_out = Eigen::VectorXd::Zero(model.nq);
  Eigen::VectorXd v_out = Eigen::VectorXd::Zero(model.nv);
  Eigen::VectorXd tau_out =
      Eigen::VectorXd::Zero(model.nv - model.joints[root_joint_id].nv());
  fromReduced(model, reduced_model, q_out, v_out, tau_out, q_in, v_in, tau_in,
              qref, locked_joint_ids);
  return {q_out, v_out, tau_out};
}

template <int Options, template <typename, int> class JointCollectionTpl>
static inline std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd>
toReduced_return(
    const pinocchio::ModelTpl<double, Options, JointCollectionTpl> &model,
    const pinocchio::ModelTpl<double, Options, JointCollectionTpl>
        &reduced_model,
    const Eigen::Ref<const Eigen::VectorXd> &q_in,
    const Eigen::Ref<const Eigen::VectorXd> &v_in,
    const Eigen::Ref<const Eigen::VectorXd> &tau_in) {
  const std::size_t root_joint_id = get_root_joint_id(model);
  Eigen::VectorXd q_out = Eigen::VectorXd::Zero(reduced_model.nq);
  Eigen::VectorXd v_out = Eigen::VectorXd::Zero(reduced_model.nv);
  Eigen::VectorXd tau_out = Eigen::VectorXd::Zero(
      reduced_model.nv - model.joints[root_joint_id].nv());
  toReduced(model, reduced_model, q_out, v_out, tau_out, q_in, v_in, tau_in);
  return {q_out, v_out, tau_out};
}

} // namespace crocoddyl_msgs

#endif // CONVERSIONS_H_
