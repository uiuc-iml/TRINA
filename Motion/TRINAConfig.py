from math import sqrt,pi
import math
try:
    import trina
except ImportError:
    import sys,os
    sys.path.append(os.path.expanduser("~/TRINA"))
    import trina

left_limb_address = trina.settings.robot_settings()['left_limb_address']
right_limb_address = trina.settings.robot_settings()['right_limb_address']
## The payload and cog has to be calibrated...
# calibrated result for robotiq gripper, in klampt frame x: array([ 0.86125143,  0.05865107,  0.00479324, -0.00341504])
# The klampt in UR EE frame rotation matrix is
# 0 -0.707   0.707
# 0 - 0.707  -0.707
# 1  0        0
##These are for the Righthand gripper
# left_limb_payload = 0.86125
# left_limb_cog = [-0.0058,-0.001,0.05865] #this needs to be in UR EE frame
left_limb_payload =1.025# 0.71#0.1#1.025(parallel)
left_limb_cog = [0.0,0.0,0.06]#0.06]

#estimated for the pusher, need to run the calibrater
right_limb_payload = 0.0
right_limb_cog = [0.0,0.0,0.0]
left_Robotiq = True
right_Robotiq = False
left_Robotiq_type = 'parallel'
right_Robotiq_type = 'parallel'
ur5e_control_rate = 0.004 #250 Hz


# left_limb_gravity_upright = [-4.91,-4.91,-6.93672]  #R_upright_newlocal * left_limb_gravity_upright = new gravity vector
# right_limb_gravity_upright = [4.91,-4.91,-6.93672]

# R_local_global_upright_left = [sqrt(0.5),-sqrt(0.25),-sqrt(0.25),-sqrt(0.5),-sqrt(0.25),-sqrt(0.25),0,sqrt(0.5),-sqrt(0.5)]
# R_local_global_upright_right = [-sqrt(0.5),-sqrt(0.25),sqrt(0.25),-sqrt(0.5),sqrt(0.25),-sqrt(0.25),0,-sqrt(0.5),-sqrt(0.5)]
# R_local_global_upright_left = [sqrt(0.5),sqrt(0.25),sqrt(0.25),-sqrt(0.5),sqrt(0.25),sqrt(0.25),0,-sqrt(0.5),sqrt(0.5)] ## this is used when dealing with gravity vector
# R_local_global_upright_right = [sqrt(0.5),-sqrt(0.25),-sqrt(0.25),sqrt(0.5),sqrt(0.25),sqrt(0.25),0,sqrt(0.5),sqrt(0.5)]


simulated_robot_control_rate = 0.004 #250Hz
limb_velocity_limits = [2.0,2.0,2.0,2.0,2.0,2.0]
limb_EE_velocity_limits = [1.0,1.0,1.0,1.0,1.0,1.0]
epsilon = 0.01
limb_position_upper_limits = [2.0*pi-epsilon,2.0*pi-epsilon,2.0*pi-epsilon,2.0*pi-epsilon,2.0*pi-epsilon,2.0*pi-epsilon]
limb_position_lower_limits = [-2.0*pi+epsilon,-2.0*pi+epsilon,-2.0*pi+epsilon,-2.0*pi+epsilon,-2.0*pi+epsilon,-2.0*pi+epsilon]
collision_check_interval = 0.1

#commonly used arm configurations for Anthrax
def mirror_arm_config(config):
    RConfig = []
    RConfig.append(-config[0])
    RConfig.append(-config[1]-math.pi)
    RConfig.append(-config[2])
    RConfig.append(-config[3]+math.pi)
    RConfig.append(-config[4])
    RConfig.append(-config[5])
    return RConfig

###
#local in global R
def get_wrench_R_left(name):
    if ((name == "anthrax")|(name == 'anthrax_lowpoly')|(name == 'seed')):
        return [sqrt(0.5),-sqrt(0.25),-sqrt(0.25),-sqrt(0.5),-sqrt(0.25),-sqrt(0.25),0,sqrt(0.5),-sqrt(0.5)]
    elif (name == "bubonic"):
        return [0.7071067811865476, 0.49999999999999994, -0.5, -0.7071067811865476, 0.49999999999999994, -0.5, 0.0, 0.7071067811865476, 0.7071067811865476]
    else:
        raise ValueError("Invalid name of robot? "+name)
    return

def get_wrench_R_right(name):
    if ((name == "anthrax")|(name == 'anthrax_lowpoly')|(name == 'seed')):
        return [sqrt(0.5),-sqrt(0.25),-sqrt(0.25),-sqrt(0.5),-sqrt(0.25),-sqrt(0.25),0,sqrt(0.5),-sqrt(0.5)]
    elif (name == "bubonic"):
        return [-0.7071067811865476, 0.49999999999999994, 0.5, -0.7071067811865476, -0.49999999999999994, -0.5, 0.0, -0.7071067811865476, 0.7071067811865476]  
    else:
        raise ValueError("Invalid name of robot? "+name)
    return

def get_base_dofs():
    return [0,1,3]

def get_left_gravity_vector_upright(name):
    if ((name == "anthrax")|(name == 'anthrax_lowpoly')|(name == 'seed')):
        return [-4.91,-4.91,-6.93672]
    elif (name == "bubonic"):
        return [-4.91,-4.91,6.93672]

def get_right_gravity_vector_upright(name):
    if ((name == "anthrax")|(name == 'anthrax_lowpoly')|(name == 'seed')):
        return [4.91,-4.91,-6.93672]
    elif (name == "bubonic"):
        return [4.91,-4.91,6.93672]    

def get_left_tool_link_N(name):
    if((name == "anthrax")|(name == "anthrax_lowpoly")|(name == "bubonic")):
        return 13
    elif((name == "seed")|(name == "half_anthrax")):
        return 16
    else:
        raise ValueError("Invalid name of robot? "+name)

def get_right_tool_link_N(name):
    if((name == "anthrax")|(name == "anthrax_lowpoly")|(name == "bubonic")):
        return 21
    elif((name == "seed")|(name == "half_anthrax")):
        return 41
    else:
        raise ValueError("Invalid name of robot? "+name)
def get_left_active_Dofs(name):
    if( (name == "anthrax")|(name == "anthrax_lowpoly")|(name == "bubonic")):
        return [7,8,9,10,11,12]
    elif((name == "seed")|(name == "half_anthrax")):
        return [10,11,12,13,14,15]
    else:
        raise ValueError("Invalid name of robot? "+name)

def get_right_active_Dofs(name):
    if((name == "anthrax")|(name == "anthrax_lowpoly")|(name == "bubonic")):
        return [15,16,17,18,19,20]
    elif((name == "seed")|(name == "half_anthrax")):
        return [35,36,37,38,39,40]
    else:
        raise ValueError("Invalid name of robot? "+name)

#TRINA_left_tool_link_N = 16
#TRINA_right_tool_link_N = 41 #seed
#TRINA_right_tool_link_N = 26 #anthrax
# TRINA_left_active_Dofs = [10,11,12,13,14,15]
# TRINA_right_active_Dofs = [35,36,37,38,39,40] #seed
#TRINA_right_active_Dofs = [20,21,22,23,24,25] #anthrax

def get_klampt_model_q(name,left_limb = [0]*6,right_limb = [0]*6,base = [0]*3):
    if((name == 'anthrax')|(name=="anthrax_lowpoly")|(name == "bubonic")):
        return base[0:2] + [0]*1 + [base[2]] + [0]*3 + left_limb + [0]*2 + right_limb + [0]
    elif(name == 'seed'):
        return base[0:3] + [0]*7 + left_limb + [0]*19 + right_limb + [0]*18
    elif(name == 'half_anthrax'):
        return base[0:3] + [0]*7 + left_limb + [0]*19 + [1.16] + [0]*5 + [0]*18 #at a position that does not collide with left limb
    else:
        print("wrong model name used.")
        return None


###To add a new robot model:
# create a new world file contained the model_name
# update the functions defined where
# when using the motion api, specify the correct model name


####
# left_limb_gravity_upright = [-4.91,-4.91,-6.93672]  #R_upright_newlocal * left_limb_gravity_upright = new gravity vector
# right_limb_gravity_upright = [4.91,-4.91,-6.93672]
##This is for the wrench convertions
# R_local_global_upright_left = [sqrt(0.5),-sqrt(0.25),-sqrt(0.25),-sqrt(0.5),-sqrt(0.25),-sqrt(0.25),0,sqrt(0.5),-sqrt(0.5)]
# R_local_global_upright_right = [-sqrt(0.5),-sqrt(0.25),sqrt(0.25),-sqrt(0.5),sqrt(0.25),-sqrt(0.25),0,-sqrt(0.5),-sqrt(0.5)]

## this is used when dealing with gravity vector
# R_URbase_global_upright_left = [sqrt(0.5),sqrt(0.25),sqrt(0.25),-sqrt(0.5),sqrt(0.25),sqrt(0.25),0,-sqrt(0.5),sqrt(0.5)] 
# R_URbase_global_upright_right = [sqrt(0.5),-sqrt(0.25),-sqrt(0.25),sqrt(0.5),sqrt(0.25),sqrt(0.25),0,sqrt(0.5),sqrt(0.5)]

