string info                                   # text info on this contact
string geom1_name                             # name of contact geom1
string geom2_name                             # name of contact geom2
geometry_msgs/Wrench[] wrenches               # list of forces/torques
geometry_msgs/Vector3[] contact_positions     # list of contact position
geometry_msgs/Vector3[] contact_normals       # list of contact normals
float64[] depths                              # list of penetration depths
