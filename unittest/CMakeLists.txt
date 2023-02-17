SET(${PROJECT_NAME}_PYTHON_TESTS
  solver_statistics
  solver_trajectory
  # whole_body_state  # not working yet
  # whole_body_trajectory  # not working yet
)

IF(CATKIN_ENABLE_TESTING)
  FIND_PACKAGE(rostest REQUIRED)
  FOREACH(TEST ${${PROJECT_NAME}_PYTHON_TESTS})
    ADD_ROSTEST(crocoddyl_msgs.test ARGS test_name:=${TEST})
  ENDFOREACH(TEST ${${PROJECT_NAME}_PYTHON_TESTS})
ENDIF(CATKIN_ENABLE_TESTING)
