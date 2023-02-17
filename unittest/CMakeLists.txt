SET(${PROJECT_NAME}_PYTHON_TESTS
  solver_statistics
  solver_trajectory
  whole_body_state
  whole_body_trajectory
)

FOREACH(TEST ${${PROJECT_NAME}_PYTHON_TESTS})
  CATKIN_ADD_NOSETESTS(test_${TEST}.py)
ENDFOREACH(TEST ${${PROJECT_NAME}_PYTHON_TESTS})

