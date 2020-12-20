import time,math
import os
import sys
try:
    import trina
except ImportError:  #run from command line?
    sys.path.append(os.path.expanduser("~/TRINA"))
    import trina
from trina import jarvis
from trina.modules.Example.api import ExampleAPI
import numpy as np

class Example(jarvis.APIModule):
    """A very simple example APIModule that implements a couple of RPC calls.
    """
    def __init__(self,Jarvis = None, debugging = False):
        jarvis.APIModule.__init__(self,Jarvis)
        self.status = 'idle' #states are " idle, active"        
        self.loopCount = 0

        self.t0 = time.time()
        self.jarvis.server.set(["Example"],{})
        #startSimpleThread will actually take care of the locking for you if dolock=True. 
        self.startSimpleThread(self._infoLoop,0.1,name="infoLoop")
        self.startSimpleThread(self._otherLoop,1.0,name="otherLoop")

    def add(self,a,b):
        if isinstance(a,list):
            a = np.asarray(a)
        if isinstance(b,list):
            b = np.asarray(b)
        res = a+b
        if isinstance(res,np.ndarray):
            return res.tolist()
        return res

    def sin(self,x):
        if isinstance(x,list):
            return np.sin(x).tolist()
        else:
            return math.sin(x)

    def sineWave(self):
        return math.sin(time.time()-self.t0)

    def apiName(self):
        return "example"

    def api(self,module_name,*args,**kwargs):
        print(args,kwargs)
        return ExampleAPI(self.apiName(),module_name,*args,**kwargs)

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

        #This is new -- added so that redisRpc calls are handled
        self.processRedisRpcs()

    def doRpc(self,fn,args,kwargs):
        #can do other things here... by default it will find fn in self and call it
        return jarvis.APIModule.doRpc(self,fn,args,kwargs)

    def _otherLoop(self):
        self.loopCount += 1
        self.jarvis.server.set(["Example","loopCount"],self.loopCount)
        
class ExampleTester(jarvis.Module):
    def __init__(self, Jarvis=None):
        jarvis.Module.__init__(self,Jarvis)
        self.count = 0
        self.startSimpleThread(self.testing,1.0,name="testingLoop")

    def testing(self):
        self.count += 1
        if self.count == 1:
            promise = self.jarvis.example.add(3,5)
            print("3 + 5 =",promise)
            t0 = time.time()
            result = promise.await()
            t1 = time.time()
            print("await 3 + 5 =",result,"returned in",t1-t0,"s")
        elif self.count == 2:
            promise = self.jarvis.example.sin([0.1,0.2,0.3,0.4])
            print("sin([0.1-0.4]) =",promise)
            t0 = time.time()
            result = promise.await()
            t1 = time.time()
            print("await sin([0.1-0.4]) =",result,"returned in",t1-t0,"s")
        elif self.count < 5:
            res = self.jarvis.example.sineWave().await()
            print("await sine wave =",res)

if __name__ == "__main__" :
    example = Example()
    tester = ExampleTester()
    #normally this will be set up by the Command Server
    tester.jarvis.server['example'] = {}
    tester.jarvis.example = example.api("tester",tester.jarvis.server)
    #for testing only. Dont terminate the program after the constructor finishes.  This will exit upon Ctrl+C
    while example.status != 'terminated':
        time.sleep(1.0)