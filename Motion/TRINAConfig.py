from math import sqrt,pi
import math
left_limb_address = '10.1.1.30'
right_limb_address = '10.1.1.20'
## The payload and cog has to be calibrated...
# calibrated result for robotiq gripper, in klampt frame x: array([ 0.86125143,  0.05865107,  0.00479324, -0.00341504])
# The klampt in UR EE frame rotation matrix is
# 0 -0.707   0.707
# 0 - 0.707  -0.707
# 1  0        0
##These are for the Righthand gripper
# left_limb_payload = 0.86125
# left_limb_cog = [-0.0058,-0.001,0.05865] #this needs to be in UR EE frame
left_limb_payload = 0.1 #1.025 
left_limb_cog = [0.,0.,0.1] #[0.0,0.0,0.08] 

#estimated for the pusher, need to run the calibrater
right_limb_payload = 0.86125 #0.4
right_limb_cog = [0.0,0.0,0.06]
left_Robotiq = True
right_Robotiq = True
left_Robotiq_type = 'parallel'
right_Robotiq_type = 'vacuum'
ur5e_control_rate = 0.004 #250 Hz

# left_limb_gravity_upright = [-4.91,-4.91,-6.93672]  #R_upright_newlocal * left_limb_gravity_upright = new gravity vector
# right_limb_gravity_upright = [4.91,-4.91,-6.93672]

# R_local_global_upright_left = [sqrt(0.5),-sqrt(0.25),-sqrt(0.25),-sqrt(0.5),-sqrt(0.25),-sqrt(0.25),0,sqrt(0.5),-sqrt(0.5)]
# R_local_global_upright_right = [-sqrt(0.5),-sqrt(0.25),sqrt(0.25),-sqrt(0.5),sqrt(0.25),-sqrt(0.25),0,-sqrt(0.5),-sqrt(0.5)]
# R_local_global_upright_left = [sqrt(0.5),sqrt(0.25),sqrt(0.25),-sqrt(0.5),sqrt(0.25),sqrt(0.25),0,-sqrt(0.5),sqrt(0.5)] ## this is used when dealing with gravity vector
# R_local_global_upright_right = [sqrt(0.5),-sqrt(0.25),-sqrt(0.25),sqrt(0.5),sqrt(0.25),sqrt(0.25),0,sqrt(0.5),sqrt(0.5)]
collision_margin = 0.0025
simulated_robot_control_rate = 0.004 #250Hz
limb_velocity_limits = [2.0,2.0,2.0,2.0,2.0,2.0]
limb_EE_velocity_limits = [1.0,1.0,1.0,1.0,1.0,1.0]
epsilon = 0.01
limb_position_upper_limits = [2.0*pi-epsilon,2.0*pi-epsilon,2.0*pi-epsilon,2.0*pi-epsilon,2.0*pi-epsilon,2.0*pi-epsilon]
limb_position_lower_limits = [-2.0*pi+epsilon,-2.0*pi+epsilon,-2.0*pi+epsilon,-2.0*pi+epsilon,-2.0*pi+epsilon,-2.0*pi+epsilon]
collision_check_interval = 0.1

#commonly used arm configurations for Anthrax
# left_untucked_config = [-0.2028,-2.1063,-1.610,3.7165,-0.9622,0.0974]
left_untucked_config = [0.14728498458862305, -1.6879149876036585, -2.212571144104004, 3.9013611513325195, -0.611485783253805, 0.0978240966796875]
def mirror_arm_config(config):
    RConfig = []
    RConfig.append(-config[0])
    RConfig.append(-config[1]-math.pi)
    RConfig.append(-config[2])
    RConfig.append(-config[3]+math.pi)
    RConfig.append(-config[4])
    RConfig.append(-config[5])
    return RConfig

right_untucked_config = mirror_arm_config(left_untucked_config)
# Bubonic
# left_untucked_config = [2.525717355599908, 2.0460428045669827, -3.920560436831199, -0.4358181444600667, -0.7994447765249488, 0.23715545920590908]
# right_untucked_config = mirror_arm_config(left_untucked_config)

left_camera_link = 15
left_camera_transform = ([0.8482332102348008, 0.058965318844127174, -0.526330231155664, -0.5277114073854315, 0.009740454714833709, -0.8493678791061241,\
     -0.04495655202583304, 0.9982125097971668, 0.039378848566075174], [0.03110821758646211, -0.03880290087626729, 0.1443583875958857])

# left_right_transform = ([-0.06939732886469613, -0.8820629769708032, -0.4659923984399842, -0.038975726714309905, 0.46915927983703953, -0.8822530605613572,\
#      -0.9968274191019911, 0.04306361341311882, 0.0669374463679997], [-0.038713496576159745, 0.022955546540886766, -0.051000723894234594])

#local in global R
def get_wrench_R_left(name):
    if ((name == "anthrax")|(name == 'anthrax_lowpoly')|(name == 'seed')|(name == 'cholera')):
        return [sqrt(0.5),-sqrt(0.25),-sqrt(0.25),-sqrt(0.5),-sqrt(0.25),-sqrt(0.25),0,sqrt(0.5),-sqrt(0.5)]
    elif (name == "bubonic"):
        return [0.7071067811865476, 0.49999999999999994, -0.5, -0.7071067811865476, 0.49999999999999994, -0.5, 0.0, 0.7071067811865476, 0.7071067811865476]

    return

def get_wrench_R_right(name):
    if ((name == "anthrax")|(name == 'anthrax_lowpoly')|(name == 'seed')|(name == 'cholera')):
        return [sqrt(0.5),-sqrt(0.25),-sqrt(0.25),-sqrt(0.5),-sqrt(0.25),-sqrt(0.25),0,sqrt(0.5),-sqrt(0.5)]
    elif (name == "bubonic"):
        return [-0.7071067811865476, 0.49999999999999994, 0.5, -0.7071067811865476, -0.49999999999999994, -0.5, 0.0, -0.7071067811865476, 0.7071067811865476]  

    return


def get_left_gravity_vector_upright(name):
    if ((name == "anthrax")|(name == 'anthrax_lowpoly')|(name == 'seed')|(name == 'cholera')):
        return [-4.91,-4.91,-6.93672]
    elif (name == "bubonic"):
        return [-4.91,-4.91,6.93672]

def get_right_gravity_vector_upright(name):
    if ((name == "anthrax")|(name == 'anthrax_lowpoly')|(name == 'seed')|(name == 'cholera')):
        return [4.91,-4.91,-6.93672]
    elif (name == "bubonic"):
        return [4.91,-4.91,6.93672]    

def get_left_tool_link_N(name):
    if((name == "anthrax")|(name == "anthrax_lowpoly")|(name == "bubonic")):
        return 13
    elif((name == "seed")|(name == "half_anthrax")):
        return 16
    elif((name == "cholera")):
        return 17

def get_right_tool_link_N(name):
    if((name == "anthrax")|(name == "anthrax_lowpoly")|(name == "bubonic")):
        return 21
    elif((name == "seed")|(name == "half_anthrax")):
        return 41
    elif((name == "cholera")):
        return 25

def get_left_active_Dofs(name):
    if( (name == "anthrax")|(name == "anthrax_lowpoly")|(name == "bubonic")):
        return [7,8,9,10,11,12]
    elif((name == "seed")|(name == "half_anthrax")):
        return [10,11,12,13,14,15]
    elif((name == "cholera")):
        return [11,12,13,14,15,16]

def get_right_active_Dofs(name):
    if((name == "anthrax")|(name == "anthrax_lowpoly")|(name == "bubonic")):
        return [15,16,17,18,19,20]
    elif((name == "seed")|(name == "half_anthrax")):
        return [35,36,37,38,39,40]
    elif((name == "cholera")):
        return [19,20,21,22,23,24]

#TRINA_left_tool_link_N = 16
#TRINA_right_tool_link_N = 41 #seed
#TRINA_right_tool_link_N = 26 #anthrax
# TRINA_left_active_Dofs = [10,11,12,13,14,15]
# TRINA_right_active_Dofs = [35,36,37,38,39,40] #seed
#TRINA_right_active_Dofs = [20,21,22,23,24,25] #anthrax

def get_klampt_model_q(name,left_limb = [0]*6,right_limb = [0]*6,base = [0]*3,head = [0]*2):
    if((name == 'anthrax')|(name=="anthrax_lowpoly")|(name == "bubonic")):
        return base[0:2] + [0]*1 + [base[2]] + [0]*3 + list(left_limb) + [0]*2 + list(right_limb) + [0]
    elif(name == 'seed'):
        return base[0:3] + [0]*7 + list(left_limb) + [0]*19 + list(right_limb) + [0]*18
    elif(name == 'half_anthrax'):
        return base[0:3] + [0]*7 + list(left_limb) + [0]*19 + [1.16] + [0]*5 + [0]*18 #at a position that does not collide with left limb
    elif(name == 'cholera'):
        return base[0:2] + [0]*1 + [base[2]] + [0]*3 + list(head) + [0]*2 + list(left_limb) + [0]*2 + list(right_limb) + [0]*13
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

