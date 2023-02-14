///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2020-2023, Heriot-Watt University, University of Oxford
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef CROCODDYL_MSG_WHOLE_BODY_STATE_H_
#define CROCODDYL_MSG_WHOLE_BODY_STATE_H_

#include <map>
#include <Eigen/Dense>

#include <pinocchio/fwd.hpp>
#include <pinocchio/spatial/motion.hpp>
#include <pinocchio/spatial/force.hpp>

// Resolves to int8
enum ContactTypeEnum { LOCOMOTION = 0, MANIPULATION = 1 };
enum ContactStateEnum { UNKNOWN = 0, OPEN = 1, CLOSED = 2, SLIPPING = 3 };

struct ContactState {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Initialize the data structure for the contact state
   */
  ContactState() {}

  /**
   * @copybrief ContactState()
   *
   * @param[in] name  Contact name
   */
  ContactState(const std::string name) : name_(name) {}

  /**
   * @copybrief ContactState()
   *
   * @param[in] name              Contact name
   * @param[in] position          Contact position
   * @param[in] velocity          Contact velocity
   * @param[in] force             Contact force
   * @param[in] surface_normal    Normal vector at the contact surface
   * @param[in] surface_friction  Friction coefficient of the contact surface
   */
  ContactState(const std::string name, const pinocchio::SE3 &position, const pinocchio::Motion &velocity,
               const pinocchio::Force &force, const Eigen::Vector3d &surface_normal, const double surface_friction)
      : name_(name),
        position(position),
        velocity(velocity),
        force(force),
        surface_normal(surface_normal),
        surface_friction(surface_friction) {}
  ~ContactState() = default;

  std::string name_;
  pinocchio::SE3 position =
      pinocchio::SE3(NAN * Eigen::Matrix3d::Identity(), Eigen::Vector3d(NAN, NAN, NAN));  ///< Contact position
  pinocchio::Motion velocity =
      pinocchio::Motion(Eigen::Vector3d(NAN, NAN, NAN), Eigen::Vector3d(NAN, NAN, NAN));  ///< Contact velocity
  pinocchio::Force force =
      pinocchio::Force(Eigen::Vector3d(NAN, NAN, NAN), Eigen::Vector3d(NAN, NAN, NAN));  ///< Contact force
  Eigen::Vector3d surface_normal = Eigen::Vector3d(NAN, NAN, NAN);  ///< Normal vector at the contact surface
  double surface_friction;                                          ///< Friction coefficient of the contact surface
  ContactTypeEnum type = ContactTypeEnum::LOCOMOTION;               ///< Contact type
  ContactStateEnum state = ContactStateEnum::UNKNOWN;               ///< Classified contact state
};

typedef std::map<std::string, ContactState> ContactStateMap;

struct WholeBodyState {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Initialize the data structure for the whole-body state
   */
  WholeBodyState() {}

  /**
   * @copybrief WholeBodyState()
   *
   * @param[in] nq  Dimension of the configuration tuple
   * @param[in] nv  Dimension of the velocity vector
   * @param[in] nu  Dimension of the control vector
   */
  WholeBodyState(const std::size_t nq, const std::size_t nv, const std::size_t nu)
      : q(Eigen::VectorXd::Zero(nq)),
        v(Eigen::VectorXd::Zero(nv)),
        a(Eigen::VectorXd::Zero(nv)),
        tau(Eigen::VectorXd(nu)) {}
  ~WholeBodyState() = default;

  double t = 0.0;                                          ///< Time from start (if part of a trajectory)
  Eigen::VectorXd q;                                       ///< Configuration vector (size nq)
  Eigen::VectorXd v;                                       ///< Tangent vector (size nv)
  Eigen::VectorXd a;                                       ///< System acceleration vector (size nv)
  Eigen::VectorXd tau;                                     ///< Torque vector (size njoints-2)
  ContactStateMap contacts;  ///< Contact state (p, pd, f, s)
                                                           // p, pd, f, s - or ContactState!
};

#endif // CROCODDYL_MSG_WHOLE_BODY_STATE_H_