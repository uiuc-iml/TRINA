import os
import sys
import time
from threading import Thread, Lock
import threading
#import the limb, base, EE, gripper,Torso, etc classes...
#import the config file for network configurations
#import TRINAConfig
#import motionStates

class Motion:
    def __init__(self):
        ##T#he individual components
        self.left_limb = armController(TRINAConfig.left_arm_address)
        self.right_limb = armController(TRINAConfig.right_arm_address)
	    #self.base = MobileBase()
        self.left_limb_state = LimbState()
        self.right_limb_state = LimbState()
        ##TODO: Add other components

        ###Other 
        self.startTime = time.time()
        self.t = 0 #time since startup
        self.startUp = False
        self.dt = 0.002
        self.lastLoopTime = 0
        self.stopMotionFlag = False
        self.stopMotionSent = False
        self.shutdownFlag = False
        self.controlLoopLock = Lock()

    def time(self):
        """Returns the time since robot startup, in s"""
        return self.t

    def startup(self):
        """After starting, all components stay where they are and update their positions immediately"""
        ###start arm
        #TODO: when we start the arms, we need to make sure that the gravity vectors are set right....
        res = self.left_arm.start()
        if res == False:
            #better to replace this with logger
            print("motion.startup(): ERROR, left limb start failure.")
            return False
        else:
            print("motion.startup(): left limb started.")

        res = self.right_arm.start()    
        if res == False:
            #better to replace this with logger
            print("motion.startup(): ERROR, left limb start failure.")
            return False
        else:
            print("motion.startup(): left limb started.")


        #start the other components        

        #overrides the default Ctrl+C behavior which kills the program
        #check this....
        #def interrupter(x,y):
        #    self.shutdown()
        #    raise KeyboardInterrupt()
        #signal.signal(signal.SIGINT,interrupter)
        controlThread = threading.Thread(target = self.controlLoop)
        controlThread.start()
        self.startUp = True
        return self.startUp

    def controlLoop(self):

        while not self.shutdownFlag:
            loopStartTime = time.time()
            self.t = time.time() - self.startTime
            ###lock the thread 
            self.controlLoopLock.acquire()

            if self.stopMotionFlag:
                if not self.stopMotionSent: #send only once to avoid drifting...
                    self.left_limb.stopMotion()
                    self.right_limb.stopMotion()
                    self.stopMotionSent = True
            else:
                ###update current state
                if left_arm.newState():
                    self.left_arm_state.sensedq = left_arm.getConfig()
                    self.left_arm_state.senseddq = left_arm.getVelocity()
                    self.left_arm_state.sensedWrench = left_arm.getWrench()
                    left_arm.markRead()
                if right_arm.newState():
                    self.right_arm_state.sensedq = left_arm.getConfig()
                    self.right_arm_state.senseddq = left_arm.getVelocity()
                    self.right_arm_state.sensedWrench = left_arm.getWrench()
                    right_arm.markRead()

                ###send commands
                if not self.left_arm_state.commandSent:
                    ###setting position will clear velocity commands
                    if self.left_arm_state.commandType == 0:
                        self.left_arm.setConfig(self.left_arm_state.commandedq)
                    elif self.left_arm_state.commandType == 1:
                        self.left_arm.setVelocity(self.left_arm_state.commandeddq)
                    self.left_arm_state.commandSent = True

                if not self.right_arm_state.commandSent:
                    ###setting position will clear velocity commands
                    if self.right_arm_state.commandType == 0:
                        self.right_arm.setConfig(self.right_arm_state.commandedq)
                    elif self.right_arm_state.commandType == 1:
                        self.right_arm.setVelocity(self.right_arm_state.commandeddq)
                    self.right_arm_state.commandSent = True


            self.controlLoopLock.release()
            elapsedTime = time.time() - loopStartTime
            self.t = time.time() - self.startTime
            if elapsedTime < self.dt:
                time.sleep(self.dt-elapsedTime)
            else:
                pass

    ###TODO
    def setPosition(self,q):
        return 
    def setleftLimbPosition(self,q):
        return 
    def setRightLimbPosition(self,q):
        return
    def sensedLeftArmPosition(self):
        return self.left_arm_state.sensedq

    #...
    #...
    #...
    #...

    ###ENDTODO

    def shutdown(self):
        """shutdown the componets... """
        left_arm.stop()
        right_arm.stop()
        #stop other components
        return 0

    def isStarted(self):
        return self.startUp

    def moving(self):
        """Returns true if the robot is currently moving."""
        return self.left_limb.moving() or self.right_limb.moving()

    def stopMotion(self):
        """Stops all motion"""
        self.stopMotionFlag = True
        self.stopMotionSent = False
        return
    def resumeMotion(self):
        """The robot is ready to take more commands"""
        self.startMotionFlag = False
        return 


if __name__=="__main__":


    print "Testing TRINA2 Motion Module..."
    robot = motion()
    if not robot.isStarted():
        res = robot.startup()
        if not res:
            raise RuntimeError("Ebolabot Motion could not be started")
    else:
        print "Robot started by another process"
        
    print "Is robot started?",robot.isStarted()
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
