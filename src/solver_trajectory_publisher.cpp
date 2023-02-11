///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2023-2023, Heriot-Watt University
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include <Eigen/Dense>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>

#include "crocoddyl_msgs/SolverTrajectory.h"
#include "crocoddyl_msgs/conversions.h"

namespace crocoddyl_msgs {

class SolverTrajectoryRosPublisher {
public:
  /**
   * @brief Initialize the solver trajectory publisher
   *
   * @param[in] topic  Topic name
   */
  SolverTrajectoryRosPublisher(
      const std::string &topic = "/crocoddyl/solver_trajectory",
      const std::string &frame = "odom") {
    ros::NodeHandle n;
    pub_.init(n, topic, 1);
    pub_.msg_.header.frame_id = frame;
    std::cout << "Ready to send solver trajectory" << std::endl;
  }
  ~SolverTrajectoryRosPublisher() = default;

  /**
   * @brief Publish a solver trajectory ROS message
   *
   * @param ts[in]      Vector of time at the beginning of the interval
   * @param dts[in]     Vector of time durations of each interval
   * @param xs[in]      Vector of states of each interval
   * @param dxs[in]     Vector of state's rate of changes of each interval
   * @param us[in]      Vector of feed-forward control parameters of each
   * interval
   * @param Ks[in]      Vector of feedback gain in the control parametrized
   * space of each interval
   * @param types[in]   Vector of control types of each interval
   * @param params[in]  Vector of control parametrizations of each interval
   */
  void publish(const std::vector<double> &ts, const std::vector<double> &dts,
               const std::vector<Eigen::VectorXd> &xs,
               const std::vector<Eigen::VectorXd> &dxs,
               const std::vector<Eigen::VectorXd> &us = {},
               const std::vector<Eigen::MatrixXd> &Ks = {},
               const std::vector<ControlType> &types = {},
               const std::vector<ControlParametrization> &params = {}) {
    if (pub_.trylock()) {
      if (ts.size() != dts.size()) {
        throw std::invalid_argument("The size of the ts vector needs to equal "
                                    "the size of the dts vector.");
      }
      if (ts.size() != xs.size()) {
        throw std::invalid_argument("The size of the ts vector needs to equal "
                                    "the size of the xs vector.");
      }
      if (ts.size() != dxs.size()) {
        throw std::invalid_argument("The size of the ts vector needs to equal "
                                    "the size of the dxs vector.");
      }
      if (us.size() != 0 && ts.size() != us.size()) {
        throw std::invalid_argument("The size of the ts vector needs to equal "
                                    "the size of the us vector.");
      }
      if (Ks.size() != 0 && ts.size() != Ks.size()) {
        throw std::invalid_argument("The size of the ts vector needs to equal "
                                    "the size of the Ks vector.");
      }
      if (types.size() != 0 && ts.size() != types.size()) {
        throw std::invalid_argument("The size of the ts vector needs to equal "
                                    "the size of the types.");
      }
      if (params.size() != 0 && ts.size() != params.size()) {
        throw std::invalid_argument("The size of the ts vector needs to equal "
                                    "the size of the params.");
      }
      pub_.msg_.header.stamp = ros::Time::now();
      for (std::size_t i = 0; i < ts.size(); ++i) {
        pub_.msg_.intervals[i].time = ts[i];
        pub_.msg_.intervals[i].duration = dts[i];
        crocoddyl_msgs::toMsg(pub_.msg_.state_trajectory[i], xs[i], dxs[i]);
        crocoddyl_msgs::toMsg(pub_.msg_.control_trajectory[i], us[i], Ks[i],
                              types[i], params[i]);
      }
      pub_.unlockAndPublish();
    }
  }

private:
  realtime_tools::RealtimePublisher<crocoddyl_msgs::SolverTrajectory> pub_;
};

} // namespace crocoddyl_msgs

PYBIND11_MODULE(solver_trajectory_ros_publisher_py, m) {
  namespace py = pybind11;
  using namespace crocoddyl_msgs;

  int argc = 0;
  char **argv = nullptr;
  ros::init(argc, argv, "solver_trajectory_ros_publisher_py",
            ros::init_options::AnonymousName);

  m.doc() = "Python interface for publishing efficiently the Crocoddyl solver "
            "trajectory in a ROS topic.";

  py::class_<SolverTrajectoryRosPublisher,
             std::unique_ptr<SolverTrajectoryRosPublisher, py::nodelete>>(
      m, "SolverTrajectoryRosPublisher")
      .def(py::init<const std::string &, const std::string &>(),
           py::arg("topic") = "/crocoddyl/solver_trajectory",
           py::arg("frame") = "odom")
      .def("publish", &SolverTrajectoryRosPublisher::publish,
           "Publish a solver trajectory ROS message.\n\n"
           ":param ts: list of initial times of each interval\n"
           ":param dts: list of time durations of each interval\n"
           ":param xs: list of states at the beginning of each interval\n"
           ":param dxs: list of state's rate of changes of each interval\n"
           ":param us: list of control parameters of each interval\n"
           ":param Ks: list of feedback gains of each interval\n"
           ":param types: list of control types\n"
           ":param params: list of control parametrizations",
           py::arg("ts"), py::arg("dts"), py::arg("xs"), py::arg("dxs"),
           py::arg("us") = std::vector<Eigen::VectorXd>(),
           py::arg("Ks") = std::vector<Eigen::MatrixXd>(),
           py::arg("types") = std::vector<ControlType>(),
           py::arg("params") = std::vector<ControlParametrization>());
}
