set(${PROJECT_NAME}_PYTHON_TESTS
    solver_statistics solver_trajectory
    # whole_body_state whole_body_trajectory  # TODO(cmastalli): Release
    # Pinocchio
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  foreach(TEST ${${PROJECT_NAME}_PYTHON_TESTS})
    add_rostest(crocoddyl_msgs.test ARGS test_name:=${TEST})
  endforeach(TEST ${${PROJECT_NAME}_PYTHON_TESTS})
else() # ROS2
  foreach(TEST ${${PROJECT_NAME}_PYTHON_TESTS})
    add_test(NAME "test_${TEST}"
             COMMAND ${PYTHON_EXECUTABLE}
                     "${PROJECT_SOURCE_DIR}/unittest/test_${TEST}.py")
  endforeach(TEST ${${PROJECT_NAME}_PYTHON_TESTS})
endif(CATKIN_ENABLE_TESTING)
