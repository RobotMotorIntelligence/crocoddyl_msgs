Crocoddyl ROS message
==============================================

## <img align="center" height="20" src="https://i.imgur.com/vAYeCzC.png"/> Introduction

This repository provides ROS messages and Python bindings for Crocoddyl's solver diagnostics (i.e., [SolverStatistics](msg/SolverStatistics.msg)), computed trajectories (i.e., [SolverTrajectory](msg/SolverTrajectory.msg)) and whole-body state (i.e., [WholeBodyState](https://github.com/loco-3d/whole_body_state_msgs/blob/master/msg/WholeBodyState.msg)). The `SolverTrajectory` message is designed to be able to retrieve the values of state, control and feedback gain along each time interval. Instead, the `WholeBodyState` message is designed to receive and display (via the [whole_body_rviz_plugin](https://github.com/loco-3d/whole_body_state_rviz_plugin)) the robot's whole-body state.

Furthermore, the Python bindings of publishers and subscribers using roscpp help to increase communication performance while still using Python.

## :penguin: Building

**crocoddyl_msgs** has the following dependencies:
* [std_msgs](http://wiki.ros.org/std_msgs)
* [whole_body_state_msgs](https://github.com/loco-3d/whole_body_state_msgs)
* [realtime_tools](http://wiki.ros.org/realtime_tools)
* [pybind11](https://pybind11.readthedocs.io/en/stable/basics.html)
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page)
* [pinocchio](https://github.com/stack-of-tasks/pinocchio)

To compile this catkin project you need to do:

	cd your_ros_ws/
	catkin build #catkin_make

## :copyright: Credits

### :writing_hand: Written by

- [Carlos Mastalli](https://romilab.org), Heriot-Watt University :uk:

### :construction_worker: With contributions from

- [Wolfgang Merkt](http://www.wolfgangmerkt.com/research/), University of Oxford :uk:

and maintained by the [Robot Motor Intelligence (RoMI)](https://romilab.org) lab.
