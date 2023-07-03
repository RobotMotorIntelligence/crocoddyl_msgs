set(${PROJECT_NAME}_PYTHON_TESTS
    solver_statistics solver_trajectory
    # whole_body_state  # not working yet whole_body_trajectory  # not working
    # yet
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  foreach(TEST ${${PROJECT_NAME}_PYTHON_TESTS})
    add_rostest(crocoddyl_msgs.test ARGS test_name:=${TEST})
  endforeach(TEST ${${PROJECT_NAME}_PYTHON_TESTS})
endif(CATKIN_ENABLE_TESTING)
