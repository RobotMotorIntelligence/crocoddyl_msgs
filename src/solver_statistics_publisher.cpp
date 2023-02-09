///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2020-2023, Heriot-Watt University, University of Oxford
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include <pybind11/pybind11.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>

#include "crocoddyl_msgs/SolverStatistics.h"

class SolverStatisticsRosPublisher {
public:
  SolverStatisticsRosPublisher(
      const std::string &topic = "/crocoddyl/solver_statistics") {
    ros::NodeHandle n;
    pub_.init(n, topic, 1);
    std::cout << "Ready to send solver statistics" << std::endl;
  }
  ~SolverStatisticsRosPublisher() = default;

  void publish(const std::size_t iterations, const double total_time,
               const double solve_time, const double cost,
               const double regularization, const double step_length,
               const double dynamic_feasibility,
               const double equality_infeasibility,
               const double inequality_infeasibility) {
    if (pub_.trylock()) {
      pub_.msg_.stamp = ros::Time::now();
      pub_.msg_.iterations = iterations;
      pub_.msg_.total_time = total_time;
      pub_.msg_.solve_time = solve_time;
      pub_.msg_.cost = cost;
      pub_.msg_.regularization = regularization;
      pub_.msg_.step_length = step_length;
      pub_.msg_.dynamic_feasibility = dynamic_feasibility;
      pub_.msg_.equality_infeasibility = equality_infeasibility;
      pub_.msg_.inequality_infeasibility = inequality_infeasibility;

      pub_.unlockAndPublish();
    }
  }

private:
  realtime_tools::RealtimePublisher<crocoddyl_msgs::SolverStatistics> pub_;
};

namespace py = pybind11;

PYBIND11_MODULE(solver_statistics_ros_publisher_py, m) {
  int argc = 0;
  char **argv = nullptr;
  ros::init(argc, argv, "solver_statistics_ros_publisher_py",
            ros::init_options::AnonymousName);

  m.doc() = "Python interface for publishing efficiently the Crocoddyl solver "
            "statistics in a ROS topic.";

  py::class_<SolverStatisticsRosPublisher,
             std::unique_ptr<SolverStatisticsRosPublisher, py::nodelete>>(
      m, "SolverStatisticsRosPublisher")
      .def(py::init<const std::string &>(),
           py::arg("topic_mpc_statistics") = "/crocoddyl/solver_statisticss")
      .def("publish", &SolverStatisticsRosPublisher::publish,
           py::arg("iterations"), py::arg("total_time"), py::arg("solve_time"),
           py::arg("cost"), py::arg("regularization"), py::arg("step_length"),
           py::arg("dynamic_feasibility"),
           py::arg("equality_infeasibility") = 0.,
           py::arg("inequality_infeasibility") = 0.);
}