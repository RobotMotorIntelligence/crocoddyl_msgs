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

namespace crocoddyl_msgs {

class SolverStatisticsRosPublisher {
public:
  /**
   * @brief Initialize the solver statistic publisher
   *
   * @param[in] topic  Topic name
   */
  SolverStatisticsRosPublisher(
      const std::string &topic = "/crocoddyl/solver_statistics") {
    ros::NodeHandle n;
    pub_.init(n, topic, 1);
    std::cout << "Ready to send solver statistics" << std::endl;
  }
  ~SolverStatisticsRosPublisher() = default;

  /**
   * @brief Publish a solver statistic ROS message
   *
   * @param iterations[in]      Number of solver iterations
   * @param totaltime[in]       Total time
   * @param solvetime[in]       Solving time
   * @param cost[in]            Total cost
   * @param regularization[in]  Regularization value
   * @param steplength[in]      Step length applied by the solver
   * @param dynfeas[in]         Dynamic feasibility
   * @param equafeas[in]        Equality constraints feasibility
   * @param ineqfeas[in]        Inequality constraints feasibility
   */
  void publish(const std::size_t iterations, const double totaltime,
               const double solvetime, const double cost,
               const double regularization, const double steplength,
               const double dynfeas, const double equafeas,
               const double ineqfeas) {
    if (pub_.trylock()) {
      pub_.msg_.stamp = ros::Time::now();
      pub_.msg_.iterations = iterations;
      pub_.msg_.total_time = totaltime;
      pub_.msg_.solve_time = solvetime;
      pub_.msg_.cost = cost;
      pub_.msg_.regularization = regularization;
      pub_.msg_.step_length = steplength;
      pub_.msg_.dynamic_feasibility = dynfeas;
      pub_.msg_.equality_feasibility = equafeas;
      pub_.msg_.inequality_feasibility = ineqfeas;
      pub_.unlockAndPublish();
    }
  }

private:
  realtime_tools::RealtimePublisher<crocoddyl_msgs::SolverStatistics> pub_;
};

} // namespace crocoddyl_msgs

PYBIND11_MODULE(solver_statistics_ros_publisher_py, m) {
  namespace py = pybind11;
  using namespace crocoddyl_msgs;

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
           py::arg("topic") = "/crocoddyl/solver_statistics")
      .def("publish", &SolverStatisticsRosPublisher::publish,
           "Publish a solver statistic ROS message.\n\n"
           ":param iterations: number of solver iterations\n"
           ":param total_time: total computation time\n"
           ":param solve_time: solving time\n"
           ":param cost: total cost\n"
           ":param regularization: regularization value\n"
           ":param step_length: step length applied by the solver\n"
           ":param dynamic_feasibility: dynamic feasibility\n"
           ":param equality_feasibility: equality constraints feasibility "
           "(default 0)\n"
           ":param inequality_feasibility: inequality constraints feasibility "
           "(default 0)",
           py::arg("iterations"), py::arg("total_time"), py::arg("solve_time"),
           py::arg("cost"), py::arg("regularization"), py::arg("step_length"),
           py::arg("dynamic_feasibility"), py::arg("equality_feasibility") = 0.,
           py::arg("inequality_feasibility") = 0.);
}