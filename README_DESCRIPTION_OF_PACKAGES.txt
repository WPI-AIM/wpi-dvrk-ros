Package virtual_master:

Just adds a node that keeps publishing messages to psm1/set_joint_states and keeps moving the right psm in daVinciGazeboSimulation little by little

Package virtual_master_service:

Just adds a node that keeps publishing messages to psm1/set_joint_states via ros service instead of ros messages in the first package above and keeps moving the left psm's three joints by keys a,s,d for corresponding joints in one direction and for keys A,S,D in the opposite direction.

Package kd_virtual_master:

Package under development, used KDL library and loads the urdf to make kinematics and dynamic tree. Further use upcoming ...
