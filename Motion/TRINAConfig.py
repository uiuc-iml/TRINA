from math import sqrt,pi
left_limb_address = '10.1.1.20'
right_limb_address = '192.168.0.178'
left_limb_payload = 2
left_limb_TCP = [0.05,0,0,0,0,0]
ur5e_control_rate = 0.004 #250 Hz
left_limb_gravity_upright = [-4.91,-4.91,-6.93672]  #R_upright_newlocal * left_limb_gravity_upright = new gravity vector
right_limb_gravity_upright = [4.91,-4.91,-6.93672]
R_local_global_upright_left = [sqrt(0.5),sqrt(0.25),sqrt(0.25),-sqrt(0.5),sqrt(0.25),sqrt(0.25),0,-sqrt(0.5),sqrt(0.5)]
R_local_global_upright_right = [sqrt(0.5),-sqrt(0.25),sqrt(0.25),sqrt(0.5),sqrt(0.25),-sqrt(0.25),0,sqrt(0.5),sqrt(0.5)]
simulated_robot_control_rate = 0.004 #250Hz
limb_velocity_limits = [2.0,2.0,2.0,2.0,2.0,2.0]
epsilon = 0.01
limb_position_upper_limits = [2.0*pi-epsilon,2.0*pi-epsilon,2.0*pi-epsilon,2.0*pi-epsilon,2.0*pi-epsilon,2.0*pi-epsilon]
limb_position_lower_limits = [-2.0*pi+epsilon,-2.0*pi+epsilon,-2.0*pi+epsilon,-2.0*pi+epsilon,-2.0*pi+epsilon,-2.0*pi+epsilon]

TRINA_left_tool_link_N = 16
TRINA_right_tool_link_N = 41 #seed
#TRINA_right_tool_link_N = 26 #anthrax
TRINA_left_active_Dofs = [10,11,12,13,14,15]
TRINA_right_active_Dofs = [35,36,37,38,39,40] #seed
#TRINA_right_active_Dofs = [20,21,22,23,24,25] #anthrax

def get_klampt_model_q(name,left_limb = [0]*6,right_limb = [0]*6,base = [0]*3):
    if name == 'anthrax':
        return base[0:2] + [0]*3 + [base[2]] + [0]*4 + left_limb + [0]*4 + right_limb + [0]*2 
    elif(name == 'seed'):
        return base[0:3] + [0]*7 + left_limb + [0]*19 + right_limb + [0]*18
    else:
        print("wrong model name used.")
        return 

