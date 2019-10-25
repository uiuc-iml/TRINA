from math import sqrt
left_limb_address = '192.168.0.207'
right_limb_address = '192.168.0.178'
ur5e_control_rate = 0.004 #250 Hz
left_limb_gravity_upright = [-4.91,-4.91,-6.93672]  #R_upright_newlocal * left_limb_gravity_upright = new gravity vector
right_limb_gravity_upright = [4.91,-4.91,-6.93672]
R_local_global_upright_left = [sqrt(0.5),sqrt(0.25),sqrt(0.25),-sqrt(0.5),sqrt(0.25),sqrt(0.25),0,-sqrt(0.5),sqrt(0.5)]
R_local_global_upright_right = [sqrt(0.5),-sqrt(0.25),sqrt(0.25),sqrt(0.5),sqrt(0.25),-sqrt(0.25),0,sqrt(0.5),sqrt(0.5)]
simulated_robot_control_rate = 0.004 #250Hz