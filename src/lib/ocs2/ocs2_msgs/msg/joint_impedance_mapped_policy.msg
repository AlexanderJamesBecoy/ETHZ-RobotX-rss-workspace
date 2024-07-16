# publish joint impedance targets and gains that have been mapped somehow from mpc policy
string[] 		 name
mpc_state[]		 ff_torque	         # ff torque, it not mpc state but of same type
float32[]		 stiffness	         # stiffness joint gains
float32[]		 damping	         # damping joint gains
mpc_state[]		 position_reference     # joint position target
mpc_state[]		 velocity_reference     # joint velocity target
float64[]               timeTrajectory         # time trajectory for stateTrajectory and inputTrajectory
