

#ifdef CROCODDYL_MSG_DISABLE_PYBIND11_WARNINGS
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunused-value"
#pragma clang diagnostic ignored "-Wdeprecated-declarations"
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#pragma clang diagnostic pop
#else
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#endif

#include "crocoddyl_msgs/solver_statistics_publisher.h"
#include "crocoddyl_msgs/solver_statistics_subscriber.h"
#include "crocoddyl_msgs/solver_trajectory_publisher.h"
#include "crocoddyl_msgs/solver_trajectory_subscriber.h"
#include "crocoddyl_msgs/whole_body_state_publisher.h"
#include "crocoddyl_msgs/whole_body_state_subscriber.h"
#include "crocoddyl_msgs/whole_body_trajectory_publisher.h"
#include "crocoddyl_msgs/whole_body_trajectory_subscriber.h"

PYBIND11_MODULE(crocoddyl_ros, m) {
  namespace py = pybind11;
  using namespace crocoddyl_msgs;

  int argc = 0;
  char **argv = nullptr;
  ros::init(argc, argv, "crocoddyl_ros", ros::init_options::AnonymousName);

  m.doc() = "Python interface for publishing and subscribing efficiently to "
            "Crocoddyl messages in ROS.";

  py::enum_<ControlType>(m, "ControlType")
      .value("EFFORT", ControlType::EFFORT)
      .value("ACCELERATION_CONTACTFORCE",
             ControlType::ACCELERATION_CONTACTFORCE)
      .export_values();

  py::enum_<ControlParametrization>(m, "ControlParametrization")
      .value("POLYZERO", ControlParametrization::POLYZERO)
      .value("POLYONE", ControlParametrization::POLYONE)
      .value("POLYTWO", ControlParametrization::POLYTWO)
      .export_values();

  py::enum_<ContactType>(m, "ContactType")
      .value("LOCOMOTION", ContactType::LOCOMOTION)
      .value("MANIPULATION", ContactType::MANIPULATION)
      .export_values();

  py::enum_<ContactStatus>(m, "ContactStatus")
      .value("UNKNOWN", ContactStatus::UNKNOWN)
      .value("SEPARATION", ContactStatus::SEPARATION)
      .value("STICKING", ContactStatus::STICKING)
      .value("SLIPPING", ContactStatus::SLIPPING)
      .export_values();

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

  py::class_<SolverStatisticsRosSubscriber,
             std::unique_ptr<SolverStatisticsRosSubscriber, py::nodelete>>(
      m, "SolverStatisticsRosSubscriber")
      .def(py::init<const std::string &>(),
           py::arg("topic") = "/crocoddyl/solver_statistics")
      .def("get_solver_statistics",
           &SolverStatisticsRosSubscriber::get_solver_statistics,
           "Get the latest solver statistic.\n\n"
           ":return: a list with the number of iterations, total time, solve\n"
           "time, cost, regularization, step legth, dynamic, equality and\n"
           "inequality feasibilities.")
      .def("has_new_msg", &SolverStatisticsRosSubscriber::has_new_msg);

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

  py::class_<SolverTrajectoryRosSubscriber,
             std::unique_ptr<SolverTrajectoryRosSubscriber, py::nodelete>>(
      m, "SolverTrajectoryRosSubscriber")
      .def(py::init<const std::string &>(),
           py::arg("topic") = "/crocoddyl/solver_trajectory")
      .def("get_solver_trajectory",
           &SolverTrajectoryRosSubscriber::get_solver_trajectory,
           "Get the latest solver trajectory.\n\n"
           ":return: a list with the vector of time at the beginning of the\n"
           "interval, its durations, initial state, state's rate of change,\n"
           "feed-forward control, feedback gain, type of control and control\n"
           "parametrization.")
      .def("has_new_msg", &SolverTrajectoryRosSubscriber::has_new_msg);

  py::class_<WholeBodyStateRosPublisher,
             std::unique_ptr<WholeBodyStateRosPublisher, py::nodelete>>(
      m, "WholeBodyStateRosPublisher")
      .def(py::init<pinocchio::Model &, const std::string &,
                    const std::string &>(),
           py::arg("model"), py::arg("topic") = "/crocoddyl/whole_body_state",
           py::arg("frame") = "odom")
      .def(py::init<pinocchio::Model &>(), py::arg("model"))
      .def("publish", &WholeBodyStateRosPublisher::publish,
           "Publish a whole-body state ROS message.\n\n"
           ":param t: time in secs\n"
           ":param q: configuration vector (dimension: model.nq)\n"
           ":param v: generalized velocity (dimension: model.nv)\n"
           ":param tau: joint effort\n"
           ":param p: contact position\n"
           ":param pd: contact velocity\n"
           ":param f: contact force, type and status\n"
           ":param s: contact surface and friction coefficient",
           py::arg("t"), py::arg("q"), py::arg("v"), py::arg("tau"),
           py::arg("p"), py::arg("pd"), py::arg("f"), py::arg("s"));

  py::class_<WholeBodyStateRosSubscriber,
             std::unique_ptr<WholeBodyStateRosSubscriber, py::nodelete>>(
      m, "WholeBodyStateRosSubscriber")
      .def(py::init<pinocchio::Model &, const std::string &,
                    const std::string &>(),
           py::arg("model"), py::arg("topic") = "/crocoddyl/whole_body_state",
           py::arg("frame") = "odom")
      .def(py::init<pinocchio::Model &>(), py::arg("model"))
      .def("get_state", &WholeBodyStateRosSubscriber::get_state,
           "Get the latest whole-body state.\n\n"
           ":return: a list with the time at the beginning of the interval,\n"
           "configuration vector, generalized velocity, joint effort\n"
           "joint effort, contact position, contact velocity, contact force\n"
           "(wrench, type and status), and contact surface (norm and friction\n"
           "coefficient).")
      .def("has_new_msg", &WholeBodyStateRosSubscriber::has_new_msg);

  py::class_<WholeBodyTrajectoryRosPublisher,
             std::unique_ptr<WholeBodyTrajectoryRosPublisher, py::nodelete>>(
      m, "WholeBodyTrajectoryRosPublisher")
      .def(py::init<pinocchio::Model &, const std::string &,
                    const std::string &, int>(),
           py::arg("model"),
           py::arg("topic") = "/crocoddyl/whole_body_trajectory",
           py::arg("frame") = "odom", py::arg("queue") = 10)
      .def(py::init<pinocchio::Model &>(), py::arg("model"))
      .def("publish", &WholeBodyTrajectoryRosPublisher::publish,
           "Publish a whole-body trajectory ROS message.\n\n"
           ":param ts: list of interval times in secs\n"
           ":param xs: list of state vectors (dimension: model.nq + model.nv)\n"
           ":param us: list of joint efforts\n"
           ":param ps: list of contact positions\n"
           ":param pds: list of contact velocities\n"
           ":param fs: list of contact forces, types and statuses\n"
           ":param ss: list of contact surfaces and friction coefficients",
           py::arg("ts"), py::arg("xs"), py::arg("us"), py::arg("ps"),
           py::arg("pds"), py::arg("fs"), py::arg("ss"));

  py::class_<WholeBodyTrajectoryRosSubscriber,
             std::unique_ptr<WholeBodyTrajectoryRosSubscriber, py::nodelete>>(
      m, "WholeBodyTrajectoryRosSubscriber")
      .def(py::init<pinocchio::Model &, const std::string &,
                    const std::string &>(),
           py::arg("model"),
           py::arg("topic") = "/crocoddyl/whole_body_trajectory",
           py::arg("frame") = "odom")
      .def(py::init<pinocchio::Model &>(), py::arg("model"))
      .def("get_trajectory", &WholeBodyTrajectoryRosSubscriber::get_trajectory,
           "Get the latest whole-body trajectory.\n\n"
           ":return: lists of time at the beginning of the intervals,\n"
           "states, joint efforts, contact positions, contact velocities,\n"
           "contact forces, types and statuses, and contact surfaces and "
           "friction\n"
           "coefficients.")
      .def("has_new_msg", &WholeBodyTrajectoryRosSubscriber::has_new_msg);
}