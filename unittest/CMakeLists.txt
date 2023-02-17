SET(${PROJECT_NAME}_PYTHON_TESTS
  solver_statistics
  solver_trajectory
  whole_body_state
  whole_body_trajectory
)

IF(CATKIN_ENABLE_TESTING)
  FIND_PACKAGE(rostest REQUIRED)
  FOREACH(TEST ${${PROJECT_NAME}_PYTHON_TESTS})
    ADD_ROSTEST(crocoddyl_msgs.test ARGS test_name:=${TEST})
  ENDFOREACH(TEST ${${PROJECT_NAME}_PYTHON_TESTS})
ENDIF(CATKIN_ENABLE_TESTING)
