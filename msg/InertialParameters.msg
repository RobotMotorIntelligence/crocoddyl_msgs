# This message describes the inertial parameters of a single rigid body.
#
# The inertial parameters of a single rigid body defined by:
#  * the name of the rigid body
#  * the inertial parameters vector [m, mc_x, mc_y, mc_z, I_{xx}, I_{xy}, I_{yy}, I_{xz}, I_{yz}, I_{zz}]^T,
#  * where mc is the first moment of inertial m*COM, and I has its origin in the frame

# Body name
string name
# This represents the vector containing the dynamic parameters
float64[10] psi
