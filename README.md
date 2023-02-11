Crocoddyl ROS message
==============================================

## <img align="center" height="20" src="https://i.imgur.com/vAYeCzC.png"/> Introduction

This repository provides ROS messages and Python bindings for Crocoddyl's solver diagnostics (i.e., [SolverStatistics](msg/SolverStatistics.msg)) and computed trajectories (i.e., [SolverTrajectory](msg/SolverTrajectory.msg)). The `SolverTrajectory` message is designed to be able to retrieve the values of state, control and feedback gain along each time interval.

Furthermore, the Python bindings of publishers and subscribers using roscpp help to increase communication performance while still using Python.

## :penguin: Building

To compile this catkin project you need to do:

	cd your_ros_ws/
	catkin build #catkin_make

## :copyright: Credits

### :writing_hand: Written by

- [Carlos Mastalli](https://romilab.org), Heriot-Watt University :uk:

### :construction_worker: With contributions from

- [Wolfgang Merkt](http://www.wolfgangmerkt.com/research/), University of Oxford :uk:

and maintained by the [Robot Motor Intelligence (RoMI)](https://romilab.org) lab.
