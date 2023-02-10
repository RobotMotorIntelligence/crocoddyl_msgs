CMAKE_MINIMUM_REQUIRED(VERSION 3.1)
PROJECT(crocoddyl_msgs)

# Set up project properties
SET(PROJECT_NAME crocoddyl_msgs)
SET(PROJECT_DESCRIPTION "Crocoddyl ROS messages")

# Print initial message
MESSAGE(STATUS "${PROJECT_DESCRIPTION}, version ${PROJECT_VERSION}")
MESSAGE(STATUS "Copyright (C) 2023 Heriot-Watt University")
MESSAGE(STATUS "All rights reserved.")
MESSAGE(STATUS "Released under the BSD 3-Clause License.")

# Find required packages
FIND_PACKAGE(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
  pybind11_catkin
)

# catkin_python_setup()

# Define messages to be generated
ADD_MESSAGE_FILES(
  DIRECTORY msg
  FILES
   TimeInterval.msg
   State.msg
   Control.msg
   FeedbackGain.msg
   SolverStatistics.msg
   SolverTrajectory.msg
)
GENERATE_MESSAGES(
  DEPENDENCIES
  std_msgs
)

# Define catkin dependencies
CATKIN_PACKAGE(
#   INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime std_msgs
)

# Build the Python interface for the whole_body_mpc_msgs interface
PYBIND11_ADD_MODULE(solver_statistics_ros_publisher_py MODULE src/solver_statistics_publisher.cpp)
TARGET_INCLUDE_DIRECTORIES(solver_statistics_ros_publisher_py PRIVATE ${catkin_INCLUDE_DIRS})# ${EIGEN3_INCLUDE_DIR} ${Boost_INCLUDE_DIRS})
TARGET_LINK_LIBRARIES(solver_statistics_ros_publisher_py PRIVATE ${catkin_LIBRARIES})# ${Boost_LIBRARIES} pinocchio::pinocchio -lrt)
SET_TARGET_PROPERTIES(solver_statistics_ros_publisher_py PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION})

# Install
INSTALL(DIRECTORY include/${PROJECT_NAME}/  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
INSTALL(TARGETS solver_statistics_ros_publisher_py LIBRARY DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION})