import os
import sys
import time
#import the limb, base, EE, gripper,Torso, etc classes...
#import the config file for network configurations
#import TRINAConfig
class Motion:
    def __init__(self):
        self.left_limb = Limb(TRINAConfig.left_arm_address)
        self.right_limb = Limb(RIGHT)
	    self.base = MobileBase()
        #self.left_ee = EndEffector(LEFT)
        #self.right_ee = EndEffector(RIGHT)
        #self.left_gripper = Gripper(LEFT)
        #self.right_gripper = Gripper(RIGHT)
        #self.head = Head()
        #self.left_mq = LimbMotionQueue(LEFT)
        #self.right_mq = LimbMotionQueue(RIGHT)
        #self.arms_mq = LimbMotionQueue(BOTH)
        #self.planner = Planner()
        #self.left_planner = LimbPlanner(LEFT)
        #self.right_planner = LimbPlanner(RIGHT )
        #self.arms_planner = LimbPlanner(BOTH)
        #self.temp = None
    def publishState(self,addr='tcp://localhost:4568'):
        """Tell the robot to publish state to an SSPP state server on the given address.
        Must be called before calling startup()"""
        return motion_lib.publishState(c_char_p(addr))
    def setKlamptModel(self,modelfn):
        """If you wish to use the motion queue or the planner, you must 
        call this before calling robot.startup(). Load the Klampt baxter.rob model
        from the given file.
        
        Not necessary if the 'klampt_model' argument was provided to setup().
        """
        return motion_lib.setKlamptModel(c_char_p(modelfn))
    def loadCalibration(self,fn):
        return motion_lib.loadCalibration(c_char_p(fn))
    def getKlamptModel(self):
        """Returns the type string of the gripper"""
        p = create_string_buffer(4096)
        res = motion_lib.getKlamptModel(p,4096)
        if not res: raise RuntimeError()
        return p.value
    def time(self):
        """Returns the time since robot startup, in s"""
        return motion_lib.getTime()
    def startup(self):
        """Starts up the robot"""
        res = motion_lib.sendStartup()
        if res == False: return False
        #overrides the default Ctrl+C behavior which kills the program
        def interrupter(x,y):
            self.shutdown()
            raise KeyboardInterrupt()
        signal.signal(signal.SIGINT,interrupter)
        return res
    def shutdown(self):
        """Shuts down the robot"""
        return motion_lib.sendShutdown()
    def isStarted(self):
        """Returns true if the robot is started"""
        return motion_lib.isStarted()
    def isTorsoEnabled(self):
        """Returns true if the Baxter torso is enabled"""
        return motion_lib.isTorsoEnabled()
    def moving(self):
        """Returns true if the robot is currently moving."""
        return self.left_mq.moving() or self.right_mq.moving()  or self.head.nodding()
    def stopMotion(self):
        """Stops all motion"""
        return motion_lib.stopMotion()    
    def enableCollisionChecking(self,enabled=True):
        """Turns on collision checking for many motion commands, including position commands, motion queue
        commands, and end effector commands."""
        return motion_lib.enableCollisionChecking(int(enabled))
    def getKlamptSensedPosition(self):
        """Retrieves the sensed configuration as a Klamp't configuration.  The Klamp't model must be set"""
        if self.temp == None:
            n = motion_lib.getKlamptNumDofs()
            if n < 0: return False
            self.temp = (c_double*n)()
        temp_p = cast(self.temp,POINTER(c_double))
        if not motion_lib.getKlamptSensedPosition(temp_p): return False;
        return [c for c in self.temp]
    def getKlamptCommandedPosition(self):
        """Retrieves the commanded configuration as a Klamp't configuration.  The Klamp't model must be set"""
        if self.temp == None:
            n = motion_lib.getKlamptNumDofs()
            if n < 0: return False
            self.temp = (c_double*n)()
        temp_p = cast(self.temp,POINTER(c_double))
        if not motion_lib.getKlamptCommandedPosition(temp_p): return False;
        return [c for c in self.temp]
    def getKlamptSensedVelocity(self):
        """Retrieves the sensed joint velocities as a Klamp't vector.  The Klamp't model must be set"""
        if self.temp == None:
            n = motion_lib.getKlamptNumDofs()
            if n < 0: return False
            self.temp = (c_double*n)()
        temp_p = cast(self.temp,POINTER(c_double))
        if not motion_lib.getKlamptSensedVelocity(temp_p): return False;
        return [c for c in self.temp]
    def getKlamptCommandedVelocity(self):
        """Retrieves the commanded joint velocities as a Klamp't vector.  The Klamp't model must be set"""
        if self.temp == None:
            n = motion_lib.getKlamptNumDofs()
            if n < 0: return False
            self.temp = (c_double*n)()
        temp_p = cast(self.temp,POINTER(c_double))
        if not motion_lib.getKlamptCommandedVelocity(temp_p): return False;
        return [c for c in self.temp]


robot = Motion()

def setup(mode=None,libpath="",klampt_model=None,server_addr=None):
    """Motion DLL setup.  Call this before calling any other
    functions in the motion module.  This will return the Motion
    object used to interface with the robot.
    
    Arguments:
    - mode: can be None, 'kinematic', 'physical', or 'client', depending on whether
      you want to simulate or operate the real robot.  If it's None, then it
      will use whatever is currently linked to by libmotion.so.  If it's client, then
      it assumes the MotionServer_x program is running, and it will send commands to
      the motion server.
    - libpath: the path to the libmotion_x.so shared library
    - klampt_model: if not None, the path to the Klampt model of the robot.
    - server_addr: if not None, the IP address of the server.  Defaults to
      'localhost'
    """
    global motion_lib,motion_mode,robot
    #check to see if setup was called before?
    if motion_lib != None:
        if motion_mode == mode:
            print "motion.setup(): Warning, already called setup before"
            return
        else:
            raise ValueError("motion.setup(): previously set up "+motion_mode+" mode, now requesting "+mode)

    motion_mode = mode

    #load the C library and set up the return types
    if mode==None:
        dllname = "motion"
    else:
        dllname = "motion_"+mode
    print "Trying to load from library path",libpath
    print "Trying to load dll named",dllname
    if platform.system() == 'Windows':
        motion_lib = windll.LoadLibrary(libpath+dllname+".dll")
    elif platform.system().startswith('CYGWIN'):
        motion_lib = cdll.LoadLibrary(libpath+"cyg"+dllname+".dll")
    else:
        motion_lib = cdll.LoadLibrary(libpath+"lib"+dllname+".so")

    if mode=='client' and server_addr != None:
        motion_lib.setServerAddr(c_char_p(server_addr))        

    #set up some return types
    motion_lib.getHeadPan.restype = c_double
    motion_lib.getMobileBaseMoveTime.restype = c_double
    motion_lib.getTime.restype = c_double
    motion_lib.getMotionQueueMoveTime.restype = c_double
    motion_lib.getGripperMoveTime.restype = c_double

    #set up the robot interface
    if klampt_model != None:
        existing_model = robot.getKlamptModel()
        if len(existing_model) > 0:
            if existing_model != klampt_model:
                raise RuntimeError("Klamp't model has already been set! "+existing_model)
        else:
            if not robot.setKlamptModel(klampt_model):
                raise RuntimeError("Error setting Klamp't model "+klampt_model)
    else:
        klampt_model = robot.getKlamptModel()

    print
    print "****** Motion Python API started *******"
    print "   Mode:",mode if mode!=None else "Default"
    print "   Klamp't model:",klampt_model
    print "******************************************"
    print 
    return robot

if __name__=="__main__":


    print "Testing Ebolabot Motion Module..."
    #assumes Klampt is in the home directory
    klampt_model = os.path.join(os.path.expanduser("~"),"Klampt/data/robots/baxter_col.rob")
    mode = None
    if len(sys.argv)>1:
        mode = sys.argv[1]
    robot = setup(mode=mode,klampt_model=klampt_model,libpath='../')
    if not robot.isStarted():
        res = robot.startup()
        if not res:
            raise RuntimeError("Ebolabot Motion could not be started")
    else:
        print "Robot started by another process"
        
    print "Is robot started?",robot.isStarted()
    print
    print "STATUS"
    print "Base:"
    print "   enabled:",robot.base.enabled()
    print "   moving:",robot.base.moving()
    print "   odometry:",robot.base.odometryPosition()
    print "   velocity:",robot.base.velocity()
    print "Left arm:"
    print "   configuration:",robot.left_limb.sensedPosition()
    print "   velocity:",robot.left_limb.sensedVelocity()
    print "   effort:",robot.left_limb.sensedEffort()
    print "   motion queue enabled:",robot.left_mq.enabled()
    print "   motion queue moving:",robot.left_mq.moving()
    print "Right arm:"
    print "   configuration:",robot.right_limb.sensedPosition()
    print "   velocity:",robot.right_limb.sensedVelocity()
    print "   effort:",robot.right_limb.sensedEffort()
    print "   motion queue enabled:",robot.right_mq.enabled()
    print "   motion queue moving:",robot.right_mq.moving()
    print "Left gripper:"
    print "   end effector xform:",robot.left_ee.sensedTransform()
    print "   type:",robot.left_gripper.type()
    print "   enabled:",robot.left_gripper.enabled()
    print "   moving:",robot.left_gripper.moving()
    print "   position:",robot.left_gripper.position()
    print "Right gripper:"
    print "   end effector xform:",robot.right_ee.sensedTransform()
    print "   type:",robot.right_gripper.type()
    print "   enabled:",robot.right_gripper.enabled()
    print "   moving:",robot.right_gripper.moving()
    print "   position:",robot.right_gripper.position()
    print
    print "Shutting down..."
    robot.shutdown()
    print "Shutdown completed"
