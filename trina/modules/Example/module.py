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
        self.loopCount = 0

        self.t0 = time.time()
        self.jarvis.server.set(["Example"],{})
        #startSimpleThread will actually take care of the locking for you if dolock=True. 
        self.startMonitorAndRpcThread(0.1)
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

    @classmethod
    def apiClass(cls):
        return ExampleAPI

    def api(self,other_module_name,other_comm_handles):
        print("Creating API for module",other_module_name)
        return ExampleAPI(other_module_name,other_comm_handles)

    def doRpc(self,fn,args,kwargs):
        #can do other things here... by default it will find fn in self and call it
        return jarvis.APIModule.doRpc(self,fn,args,kwargs)

    def _otherLoop(self):
        self.loopCount += 1
        self.jarvis.server.set(["Example","loopCount"],self.loopCount)
        

class ExampleTester(jarvis.Module):
    def __init__(self, Jarvis=None):
        jarvis.Module.__init__(self,Jarvis)
        #Request access to the Example API -- normally the Jarvis instance will be provided by the CommandServer
        res = self.jarvis.require('Example')
        assert res != None
        self.count = 0
        self.jarvis.enableHealthChecks(False)   #don't turn me off due to health problems
        self.startSimpleThread(self.testing,1.0,name="testingLoop")

    def testing(self):
        self.count += 1
        if self.count == 1:
            promise = self.jarvis.example.add(3,5)
            print("3 + 5 =",promise)
            t0 = time.time()
            result = promise.wait()
            t1 = time.time()
            print("wait 3 + 5 =",result,"returned in",t1-t0,"s")
        elif self.count == 2:
            promise = self.jarvis.example.sin([0.1,0.2,0.3,0.4])
            print("sin([0.1-0.4]) =",promise)
            t0 = time.time()
            result = promise.wait()
            t1 = time.time()
            print("wait sin([0.1-0.4]) =",result,"returned in",t1-t0,"s")
        elif self.count < 5:
            res = self.jarvis.example.sineWave().wait()
            print("wait sine wave =",res)


if __name__ == "__main__" :
    example = Example()
    tester = ExampleTester()
    #for testing only. Dont terminate the program after the constructor finishes.  This will exit upon Ctrl+C
    while example.status != 'terminated':
        time.sleep(1.0)
