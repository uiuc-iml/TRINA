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
left_limb_payload = 0.0
left_limb_cog = [0.0,0.0,0.0]

#estimated for the pusher, need to run the calibrater
right_limb_payload = 0.4
right_limb_cog = [0.0,0.0,0.06]
left_Robotiq = False
right_Robotiq = False
left_Robotiq_type = 'vacuum'
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
left_untucked_config = [-0.2028,-2.1063,-1.610,3.7165,-0.9622,0.0974]
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
left_tabletop_config = [-1.2396209875689905, -2.6281658611693324, -1.0119028091430664, 5.456587779312887, -2.3149259726153772, 0.15132474899291992]
right_tabletop_config = mirror_arm_config(left_tabletop_config)

##Calibration configs
right_calibration_configs = [[0.11264324188232422, -0.9664667409709473, 1.818247143422262, -0.852069692020752, 3.9004664421081543, -0.7985737959491175],\
    [-0.043907944356099904, -0.9470680517009278, 2.072792355214254, -0.8519972127727051, 3.9007532596588135, -0.7984898726092737],\
    [-0.03166181245912725, -1.0096052449992676, 1.4896205107318323, -0.9153650564006348, 3.9385504722595215, -2.268669907246725],\
    [-0.029310528432027638, -1.1644295018962403, 1.929971996937887, -0.8524416250041504, 3.9007651805877686, -0.79840594926943],\
    [0.15830087661743164, -1.323782281284668, 1.9610980192767542, -1.007603959446289, 4.145726203918457, -0.8638504187213343],\
    [0.2823519706726074, -1.2218292516520997, 1.738687817250387, -1.0080841344645997, 4.204855918884277, -0.9476984182940882],\
    [0.47351837158203125, -1.4614766401103516, 1.859293286000387, -0.7582662862590333, 4.250025749206543, -1.0178893248187464],\
    [-0.03166181245912725, -1.0096052449992676, 1.489620510731, -0.9153650564006348, 3.9385504722595215, -2.2686697246725],\
    [0.3085155487060547, -1.176612214451172, 2.1993735472308558, -1.5070932668498536, 4.322521686553955, -0.531555477772848],\
    [0.5120043754577637, -0.6981123250773926, 1.8136675993548792, -1.1633261007121583, 3.9039573669433594, -2.6684110800372522],\
    [0.16982078552246094, -1.205845670109131, 2.1413844267474573, -1.205829457645752, 3.917790651321411, -1.5816596190081995],\
    [0.1910691261291504, -1.1021698278239747, 1.8236640135394495, -1.0596426290324708, 3.9707016944885254, -3.328471008931295],\
    [0.00789499282836914, -0.7851079267314454, 1.6004775206195276, -0.9146102231791993, 3.938478469848633, -3.2807253042804163],\
    [0.7273612022399902, -0.36806757867846684, 1.6274054686175745, -0.9103096288493653, 3.683013439178467, -3.2802208105670374],\
    [0.5704712867736816, -0.23182375848803716, 1.6262686888324183, -0.9120701116374512, 3.8526318073272705, -2.4649649302112024],\
    [0.5688657760620117, -0.17466290414843755, 1.6044066588031214, -0.9148858350566407, 3.7006826400756836, -2.2003987471209925],\
    [0.6675128936767578, -0.3591383260539551, 1.6013758818255823, -0.913867787723877, 4.108368396759033, -1.5732596556292933],\
    [0.7430663108825684, -0.2564423841289063, 1.601328198109762, -0.9140232366374512, 4.108428478240967, -0.9232428709613245]]

fixed_calibration_configs = [[0.7858576774597168, 0.3395234781452636, -0.4835023880004883, -1.129763440494873, 2.0836281776428223, -0.29293853441347295],\
    [0.7876238822937012, 0.5427781778522949, -0.8536539077758789, -1.102635220890381, 1.706113338470459, -0.4081376234637659],\
    [0.7876238822937012, 0.5427781778522949, -0.8536539077758789, -1.102635220890381, 1.706113338470459, -0.4081376234637659],\
    [0.7667994499206543, 0.34439389287915034, -0.6551532745361328, -0.8170297902873536, 2.076389789581299, -0.6739485899554651],\
    [0.7689743041992188, 0.6103879648395996, -1.1951065063476562, -0.5441764158061524, 2.077348232269287, -0.6761840025531214],\
    [0.6562056541442871, 0.7695600229450683, -1.3006315231323242, -1.0830553335002442, 2.073681354522705, -0.5479415098773401],\
    [0.6682453155517578, 0.7613040643879394, -1.302093505859375, -1.070455865269043, 2.1790337562561035, -0.5672543684588831],\
    [0.6777515411376953, 0.6079389291950683, -1.0026273727416992, -1.2231677335551758, 2.178626537322998, -0.578841511403219],\
    [0.6880507469177246, 0.29529492437329097, -0.3745298385620117, -1.5458014768413086, 2.1780991554260254, -0.5913499037372034],\
    [0.6058783531188965, 0.39253918706860347, -0.40106678009033203, -1.559147672062256, 2.1801838874816895, -0.49125367799867803],\
    [0.5946269035339355, 0.6075097757526855, -0.8237085342407227, -1.3436811727336426, 2.180123805999756, -0.47758323351015264],\
    [0.582514762878418, 0.7473127084919433, -1.0885639190673828, -1.2101963323405762, 2.179919719696045, -0.4627755323993128],\
    [0.49260568618774414, 0.8736516672321777, -1.1660490036010742, -1.196690396671631, 2.1754040718078613, -0.35347253481020147],\
    [0.5093803405761719, 0.7556287485310058, -0.9516134262084961, -1.3164375585368653, 2.5289406776428223, -0.3822138945208948],\
    [0.5199952125549316, 0.6568185526081542, -0.7665700912475586, -1.4178341192058106, 2.529456615447998, -0.4006846586810511],\
    [0.4620037078857422, 0.8748313623615722, -1.114419937133789, -1.2060693067363282, 2.5245699882507324, -0.3003795782672327],\
    [0.47318315505981445, 0.8033057886311035, -0.985896110534668, -1.2786978048137208, 2.525911808013916, -0.31955176988710576],\
    [0.4765739440917969, 0.7613765436359863, -0.9030475616455078, -1.3246677678874512, 2.529384136199951, -0.32714873949159795]]

###
#local in global R
def get_wrench_R_left(name):
    if ((name == "anthrax")|(name == 'anthrax_lowpoly')|(name == 'seed')):
        return [sqrt(0.5),-sqrt(0.25),-sqrt(0.25),-sqrt(0.5),-sqrt(0.25),-sqrt(0.25),0,sqrt(0.5),-sqrt(0.5)]
    elif (name == "bubonic"):
        return [0.7071067811865476, 0.49999999999999994, -0.5, -0.7071067811865476, 0.49999999999999994, -0.5, 0.0, 0.7071067811865476, 0.7071067811865476]

    return

def get_wrench_R_right(name):
    if ((name == "anthrax")|(name == 'anthrax_lowpoly')|(name == 'seed')):
        return [sqrt(0.5),-sqrt(0.25),-sqrt(0.25),-sqrt(0.5),-sqrt(0.25),-sqrt(0.25),0,sqrt(0.5),-sqrt(0.5)]
    elif (name == "bubonic"):
        return [-0.7071067811865476, 0.49999999999999994, 0.5, -0.7071067811865476, -0.49999999999999994, -0.5, 0.0, -0.7071067811865476, 0.7071067811865476]  

    return


def get_left_gravity_vector_upright(name):
    if ((name == "anthrax")|(name == 'anthrax_lowpoly')|(name == 'seed')):
        return [-4.91,-4.91,-6.93672]
    elif (name == "bubonic"):
        return [-4.91,-4.91,6.93672]

def get_right_gravity_vector_upright(name):
    if ((name == "anthrax")|(name == 'anthrax_lowpoly')|(name == 'seed')):
        return [-4.91,4.91,-6.93672]
    elif (name == "bubonic"):
        return [-4.91,4.91,6.93672]    

def get_left_tool_link_N(name):
    if((name == "anthrax")|(name == "anthrax_lowpoly")):
        return 13
    elif((name == "seed")|(name == "half_anthrax")):
        return 16

def get_right_tool_link_N(name):
    if((name == "anthrax")|(name == "anthrax_lowpoly")):
        return 21
    elif((name == "seed")|(name == "half_anthrax")):
        return 41
def get_left_active_Dofs(name):
    if( (name == "anthrax")|(name == "anthrax_lowpoly")):
        return [7,8,9,10,11,12]
    elif((name == "seed")|(name == "half_anthrax")):
        return [10,11,12,13,14,15]

def get_right_active_Dofs(name):
    if((name == "anthrax")|(name == "anthrax_lowpoly")):
        return [15,16,17,18,19,20]
    elif((name == "seed")|(name == "half_anthrax")):
        return [35,36,37,38,39,40]

#TRINA_left_tool_link_N = 16
#TRINA_right_tool_link_N = 41 #seed
#TRINA_right_tool_link_N = 26 #anthrax
# TRINA_left_active_Dofs = [10,11,12,13,14,15]
# TRINA_right_active_Dofs = [35,36,37,38,39,40] #seed
#TRINA_right_active_Dofs = [20,21,22,23,24,25] #anthrax

def get_klampt_model_q(name,left_limb = [0]*6,right_limb = [0]*6,base = [0]*3):
    if((name == 'anthrax')|(name=="anthrax_lowpoly")):
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

