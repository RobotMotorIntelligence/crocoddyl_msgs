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
FIND_PACKAGE(Eigen3 REQUIRED)
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
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime std_msgs
)

# Define the Python modules
SET(${PROJECT_NAME}_PYBIND11_MODULE
  solver_statistics_publisher
  solver_statistics_subscriber
  solver_trajectory_publisher
  solver_trajectory_subscriber
)

# Build the Python interface for the ROS publisher and subscriber
FOREACH(PYBIND11_MODULE ${${PROJECT_NAME}_PYBIND11_MODULE})
  PYBIND11_ADD_MODULE(${PYBIND11_MODULE}_py MODULE src/${PYBIND11_MODULE}.cpp)
  TARGET_INCLUDE_DIRECTORIES(${PYBIND11_MODULE}_py PRIVATE include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
  TARGET_LINK_LIBRARIES(${PYBIND11_MODULE}_py PRIVATE ${catkin_LIBRARIES})
  SET_TARGET_PROPERTIES(${PYBIND11_MODULE}_py PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION})
  INSTALL(TARGETS ${PYBIND11_MODULE}_py LIBRARY DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION})
ENDFOREACH(PYBIND11_MODULE ${${PROJECT_NAME}_PYBIND11_MODULE})

# Install the include files
INSTALL(DIRECTORY include/${PROJECT_NAME}/  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})