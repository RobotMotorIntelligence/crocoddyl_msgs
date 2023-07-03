cmake_minimum_required(VERSION 3.1)
project(crocoddyl_msgs)

# Set up project properties
set(PROJECT_NAME crocoddyl_msgs)
set(PROJECT_DESCRIPTION "Crocoddyl ROS messages")

# Print initial message
message(STATUS "${PROJECT_DESCRIPTION}, version ${PROJECT_VERSION}")
message(
  STATUS "Copyright (C) 2020-2023 Heriot-Watt University, University of Oxford")
message(STATUS "All rights reserved.")
message(STATUS "Released under the BSD 3-Clause License.")

# Find required packages
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(
  catkin REQUIRED COMPONENTS roscpp std_msgs whole_body_state_msgs
                             message_generation realtime_tools pybind11_catkin)

# Define messages to be generated
add_message_files(
  DIRECTORY
  msg
  FILES
  TimeInterval.msg
  State.msg
  Control.msg
  FeedbackGain.msg
  SolverStatistics.msg
  SolverTrajectory.msg)
generate_messages(DEPENDENCIES std_msgs)

# Define catkin dependencies
catkin_package(
  INCLUDE_DIRS
  include
  CATKIN_DEPENDS
  message_runtime
  std_msgs
  whole_body_state_msgs
  DEPENDS)

# Define the Python modules
set(${PROJECT_NAME}_PYBIND11_MODULE crocoddyl_ros)

if(APPLE AND ${pybind11_catkin_VERSION} VERSION_LESS "2.10.3")
  add_definitions(-DCROCODDYL_MSG_DISABLE_PYBIND11_WARNINGS)
endif()

# Build the Python interface for the ROS publisher and subscriber
add_compile_options(-Wall -Wpedantic -Wextra -Wno-sign-conversion)
foreach(PYBIND11_MODULE ${${PROJECT_NAME}_PYBIND11_MODULE})
  pybind11_add_module(${PYBIND11_MODULE} MODULE src/${PYBIND11_MODULE}.cpp)
  target_include_directories(
    ${PYBIND11_MODULE} PRIVATE include ${catkin_INCLUDE_DIRS}
                               ${EIGEN3_INCLUDE_DIR} ${Boost_INCLUDE_DIRS})
  target_link_libraries(
    ${PYBIND11_MODULE} PRIVATE ${catkin_LIBRARIES} ${Boost_LIBRARIES}
                               ${Boost_PYTHON_LIBRARY} pinocchio::pinocchio)
  add_dependencies(${PYBIND11_MODULE} ${crocoddyl_msgs_EXPORTED_TARGETS})
  set_target_properties(
    ${PYBIND11_MODULE}
    PROPERTIES LIBRARY_OUTPUT_DIRECTORY
               ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION})
  install(TARGETS ${PYBIND11_MODULE}
          LIBRARY DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION})
endforeach(PYBIND11_MODULE ${${PROJECT_NAME}_PYBIND11_MODULE})

# Install the include files
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

add_subdirectory(unittest)
