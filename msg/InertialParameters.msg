# This message describes the inertial parameters of a single rigid body.
#
# The inertial parameters of a single rigid body defined by:
#  * the name of the rigid body
#  * the inertial parameters vector [m, h_x, h_y, h_z, I_{xx}, I_{xy}, I_{yy}, I_{xz}, I_{yz}, I_{zz}]^T,
#  * where h=mc is the first moment of inertial m*COM
#  * where I has its origin in the frame, I = I_C + mS^T(c)S(c) and I_C has its origin at the barycenter

# Body name
string name
# Body inertia
geometry_msgs/Inertia inertia
