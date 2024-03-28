# This message describes the inertial parameters of a rigid body system.
#
# A rigid body system consists of multiple bodies connected 
# through rigid joints. The description of each body's inertia is
# defined in their inertia messages.
#
# The header specifies the time at which the state is published.

Header header

# The inertia of multiple bodies
crocoddyl_msgs/InertialParameters[] parameters