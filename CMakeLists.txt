cmake_minimum_required(VERSION 3.3)
project(crocoddyl_msgs)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-sign-conversion)
endif()

# Set up project properties
set(PROJECT_NAME crocoddyl_msgs)
set(PROJECT_DESCRIPTION "Crocoddyl ROS messages")
set(PROJECT_VERSION 1.3.1)

# Print initial message
message(STATUS "${PROJECT_DESCRIPTION}, version ${PROJECT_VERSION}")
message(
  STATUS "Copyright (C) 2020-2024 Heriot-Watt University, University of Oxford")
message(STATUS "All rights reserved.")
message(STATUS "Released under the BSD 3-Clause License.")

# Find required packages
find_package(Eigen3 REQUIRED)
find_package(pinocchio 2.6.18 REQUIRED)

# Define the Python modules
set(${PROJECT_NAME}_PYBIND11_MODULE crocoddyl_ros)

if($ENV{ROS_VERSION} EQUAL 2) # ROS 2
  add_definitions(-DROS2)
  # Default to C++14
  if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
  endif()
  # Find ROS2 required packages
  set(PACKAGE_DEPENDENCIES rclcpp std_msgs whole_body_state_msgs realtime_tools
                           pybind11)
  find_package(ament_cmake REQUIRED)
  find_package(rosidl_default_generators REQUIRED)
  foreach(DEPENDENCY IN ITEMS ${PACKAGE_DEPENDENCIES})
    find_package(${DEPENDENCY} REQUIRED)
  endforeach()
  # Define messages to be generated
  rosidl_generate_interfaces(
    ${PROJECT_NAME}
    "msg/TimeInterval.msg"
    "msg/State.msg"
    "msg/Control.msg"
    "msg/FeedbackGain.msg"
    "msg/SolverStatistics.msg"
    "msg/SolverTrajectory.msg"
    "msg/InertialParameters.msg"
    "msg/MultibodyInertialParameters.msg"
    DEPENDENCIES
    std_msgs
    whole_body_state_msgs)
  # Add warning definitions
  if(APPLE AND ${pybind11_VERSION} VERSION_LESS "2.10.3")
    add_definitions(-DCROCODDYL_MSG_DISABLE_PYBIND11_WARNINGS)
  endif()
  # Build the Python interface for the ROS publisher and subscriber
  foreach(PYBIND11_MODULE ${${PROJECT_NAME}_PYBIND11_MODULE})
    pybind11_add_module(${PYBIND11_MODULE} MODULE src/${PYBIND11_MODULE}.cpp)
    target_include_directories(
      ${PYBIND11_MODULE}
      PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
             ${EIGEN3_INCLUDE_DIR} $<INSTALL_INTERFACE:include>)
    ament_target_dependencies(${PYBIND11_MODULE} PUBLIC ${PACKAGE_DEPENDENCIES})
    rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME}
                                  rosidl_typesupport_cpp)
    target_link_libraries(
      ${PYBIND11_MODULE} PUBLIC ${cpp_typesupport_target} ${Boost_LIBRARIES}
                                ${Boost_PYTHON_LIBRARY} pinocchio::pinocchio)
    set_target_properties(${PYBIND11_MODULE} PROPERTIES LIBRARY_OUTPUT_DIRECTORY
                                                        lib/${PROJECT_NAME})
    install(TARGETS ${PYBIND11_MODULE}
            LIBRARY DESTINATION ${PYTHON_INSTALL_DIR})
  endforeach(PYBIND11_MODULE ${${PROJECT_NAME}_PYBIND11_MODULE})
  # Install the include files
  install(DIRECTORY include/${PROJECT_NAME}/
          DESTINATION include/${PROJECT_NAME})
  ament_export_dependencies(rosidl_default_runtime)
  ament_package()
else() # ROS 1
  # Find ROS1 required packages
  find_package(
    catkin REQUIRED
    COMPONENTS roscpp std_msgs whole_body_state_msgs message_generation
               realtime_tools pybind11_catkin)
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
    SolverTrajectory.msg
    InertialParameters.msg
    MultibodyInertialParameters.msg)
    generate_messages(DEPENDENCIES std_msgs whole_body_state_msgs)
  # Define catkin dependencies
  catkin_package(
    INCLUDE_DIRS
    include
    CATKIN_DEPENDS
    message_runtime
    std_msgs
    whole_body_state_msgs
    DEPENDS)
  # Add warnings definition
  if(APPLE AND ${pybind11_catkin_VERSION} VERSION_LESS "2.10.3")
    add_definitions(-DCROCODDYL_MSG_DISABLE_PYBIND11_WARNINGS)
  endif()
  # Build the Python interface for the ROS publisher and subscriber
  foreach(PYBIND11_MODULE ${${PROJECT_NAME}_PYBIND11_MODULE})
    pybind11_add_module(${PYBIND11_MODULE} MODULE src/${PYBIND11_MODULE}.cpp)
    target_include_directories(
      ${PYBIND11_MODULE} PUBLIC include ${catkin_INCLUDE_DIRS}
                                ${EIGEN3_INCLUDE_DIR} ${Boost_INCLUDE_DIRS})
    target_link_libraries(
      ${PYBIND11_MODULE} PUBLIC ${catkin_LIBRARIES} ${Boost_LIBRARIES}
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
endif()

add_subdirectory(unittest)
