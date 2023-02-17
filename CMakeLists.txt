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
FIND_PACKAGE(pinocchio REQUIRED)
FIND_PACKAGE(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  whole_body_state_msgs
  message_generation
  realtime_tools
  pybind11_catkin
)

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
  CATKIN_DEPENDS message_runtime std_msgs whole_body_state_msgs
  DEPENDS
)

# Define the Python modules
SET(${PROJECT_NAME}_PYBIND11_MODULE
  crocoddyl_ros
)

# Build the Python interface for the ROS publisher and subscriber
ADD_COMPILE_OPTIONS(-Wall -Wpedantic -Wextra -Wno-sign-conversion)
FOREACH(PYBIND11_MODULE ${${PROJECT_NAME}_PYBIND11_MODULE})
  PYBIND11_ADD_MODULE(${PYBIND11_MODULE} MODULE src/${PYBIND11_MODULE}.cpp)
  TARGET_INCLUDE_DIRECTORIES(${PYBIND11_MODULE} PRIVATE include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${Boost_INCLUDE_DIRS})
  TARGET_LINK_LIBRARIES(${PYBIND11_MODULE} PRIVATE ${catkin_LIBRARIES} ${Boost_LIBRARIES} pinocchio::pinocchio)
  ADD_DEPENDENCIES(${PYBIND11_MODULE} ${crocoddyl_msgs_EXPORTED_TARGETS})
  SET_TARGET_PROPERTIES(${PYBIND11_MODULE} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION})
  INSTALL(TARGETS ${PYBIND11_MODULE} LIBRARY DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION})
ENDFOREACH(PYBIND11_MODULE ${${PROJECT_NAME}_PYBIND11_MODULE})

# Install the include files
INSTALL(DIRECTORY include/${PROJECT_NAME}/  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

ADD_SUBDIRECTORY(unittest)