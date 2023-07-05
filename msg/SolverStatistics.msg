# This message provides essential diagnostics information for Crocoddyl solvers.

# This is a message that holds data to diagnostics Crocoddyl solvers.
#
# The solver statistics is defined by:
#  * the solver iterations,
#  * the total computation time,
#  * the total solving time,
#  * the total cost,
#  * the solver regularization,
#  * the applied step length,
#  * the dynamic feasibility,
#  * the equality constraints feasibility, and
#  * the inequality constraints feasibility.

# This represents the time and frame at which the message is published
std_msgs/Header header

uint16 iterations
float32 total_time
float32 solve_time
float32 cost
float32 regularization
float32 step_length
float32 dynamic_feasibility
float32 equality_feasibility
float32 inequality_feasibility
