import time,math
import os
from threading import Thread,Lock
import sys
try:
    import trina
except ImportError:
    sys.path.append(os.path.expanduser("~/TRINA"))
    trina
from trina import jarvis

class Example(jarvis.Module):
    """A very simple example file that uses the jarvis.Module setup
    and two threads.  One thread just monitors the health of the Module
    and the other counts every second and prints the robot's commanded
    position.
    """
    def __init__(self,Jarvis = None, debugging = False):
        jarvis.Module.__init__(self,Jarvis)
        res = self.jarvis.require('Motion')  #requesting 'Motion' will provide the jarvis.robot accessor API.
        assert res != None
        self.status = 'idle' #states are " idle, active"  
        self.loopCount = 0

        #this infoloop does basically the same thing as the built-in monitor thread
        self.startMonitorThread(0.1)
        #self.startSimpleThread(self._infoLoop,0.1,name="infoLoop")
        #startSimpleThread will take care of the locking for you if dolock=True.
        self.startSimpleThread(self._otherLoop,1.0,name="otherLoop")

    def _infoLoop(self):
        self.jarvis.log_health()
        loop_start_time = time.time()
        status = self.jarvis.getActivityStatus()

        if status == 'active':
            if data.status == 'idle':
                print('\n\n\n\n starting up Example Module! \n\n\n\n\n')
                self.status = 'active'

        elif status == 'idle':
            if self.status == 'active':
                print('loop count =',self.loopCount)
                print('\n\n\n\n deactivating Example Module. \n\n\n\n\n')
                self.status = 'idle'

    def _otherLoop(self):
        print("Module status",self.status)
        self.loopCount += 1
        print("Robot commanded position:",self.jarvis.robot.getKlamptCommandedPosition())
        if self.loopCount >= 5:
            self.terminate()
            print("All done")



class ExampleSharedData:
    """Use self.lock to manage any access to data if shared between threads.
    (This is not needed if your app only uses one thread.)
    """
    def __init__(self):
        self.lock = Lock()
        self.jarvis = None
        self.status = None
        self.loopCount = None

class ExampleManualLocking(jarvis.Module):
    """A version of Example that does its own locking.  This is useful if a
    a thread takes extra time. """
    def __init__(self,Jarvis = None, debugging = False):
        jarvis.Module.__init__(self,Jarvis)
        res = self.jarvis.require('Motion')  #requesting 'Motion' will provide the jarvis.robot accessor API.
        assert res != None
        self.sharedData = ExampleSharedData()
        self.sharedData.jarvis = self.jarvis
        self.sharedData.status = 'idle' #states are " idle, active"        
        self.sharedData.loopCount = 0

        #If dolock=True, the startSimpleThread will lock access to self during each loop.
        #Since _otherLoop waits for 1 useless seconds, it would otherwise block _infoLoop
        #from completing.
        self.startSimpleThread(self._infoLoop,0.1,args=(self.sharedData,),name="infoLoop",dolock=False)
        self.startSimpleThread(self._otherLoop,1.0,args=(self.sharedData,),name="otherLoop",dolock=False)

    def _infoLoop(self,data):
        with data.lock:
            data.jarvis.log_health()
            loop_start_time = time.time()
            status = data.jarvis.getActivityStatus()

            if status == 'active':
                if data.status == 'idle':
                    print('\n\n\n\n starting up Example Module! \n\n\n\n\n')
                    data.status = 'active'

            elif status == 'idle':
                if data.status == 'active':
                    print('loop count =',data.loopCount)
                    print('\n\n\n\n deactivating Example Module. \n\n\n\n\n')
                    data.status = 'idle'

    def _otherLoop(self,data):
        with data.lock:
            print("Module status",data.status)
            data.loopCount += 1
            print("Robot commanded position:",data.jarvis.robot.getKlamptCommandedPosition())
            if data.loopCount >= 5:
                self.terminate()
        #take up a bunch of time
        time.sleep(1.0)

if __name__ == "__main__" :
    #example = Example()
    example = ExampleManualLocking()
    #for testing only. Dont terminate the program after the constructor finishes.  This will exit upon Ctrl+C
    while example.status != 'terminated':
        time.sleep(1.0)