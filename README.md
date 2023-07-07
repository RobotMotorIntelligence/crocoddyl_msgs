# Crocoddyl ROS messages

## <img align="center" height="20" src="https://i.imgur.com/vAYeCzC.png"/> Introduction

This repository provides ROS messages and Python bindings for Crocoddyl's solver diagnostics (i.e., [SolverStatistics](msg/SolverStatistics.msg)), computed trajectories (i.e., [SolverTrajectory](msg/SolverTrajectory.msg)) and whole-body state and trajectory (i.e., [WholeBodyState](https://github.com/loco-3d/whole_body_state_msgs/blob/master/msg/WholeBodyState.msg) and [WholeBodyTrajectory](https://github.com/loco-3d/whole_body_state_msgs/blob/master/msg/WholeBodyTrajectory.msg), respectively). The **`SolverTrajectory`** message is designed to be able to retrieve the values of state, control and feedback gain along each time interval. Instead, the **`WholeBodyState`** and **`WholeBodyTrajectory`** messages are designed to receive and display (via the [whole_body_rviz_plugin](https://github.com/loco-3d/whole_body_state_rviz_plugin)) the robot's whole-body state.

Furthermore, the Python bindings of publishers and subscribers using roscpp help to increase communication performance while still using Python.

## :penguin: Building

**crocoddyl_msgs** has the following dependencies:

- [std_msgs](http://wiki.ros.org/std_msgs)
- [whole_body_state_msgs](https://github.com/loco-3d/whole_body_state_msgs)
- [realtime_tools](http://wiki.ros.org/realtime_tools)
- [pybind11](https://pybind11.readthedocs.io/en/stable/basics.html) (we wrap it in ROS1 via [pybind11_catkin](https://github.com/wxmerkt/pybind11_catkin))
- [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page)
- [pinocchio](https://github.com/stack-of-tasks/pinocchio)

To compile this catkin/colcon project you need to do:

```bash
    cd your_ros_ws/
    catkin build # ROS1
    colcon build # ROS2
```

Note that this package supports ROS1 and ROS2.

## :copyright: Credits

### :writing_hand: Written by

- [Carlos Mastalli](https://romilab.org), Heriot-Watt University :uk:
- [Wolfgang Merkt](http://www.wolfgangmerkt.com/research/), University of Oxford :uk:

and maintained by the [Robot Motor Intelligence (RoMI)](https://romilab.org) lab.
