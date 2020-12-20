from threading import Thread, RLock
import threading
from multiprocessing import Process, Manager, Pipe
import time
import sys
import time
from ..utils import TimedLooper

THREAD_TIMEOUT = 30.0

class Module:
    """Base class for Jarvis modules.  Provides some convenient functions for
    starting threads and processes.

    Your app should implement name(), and in __init__ it should start the module
    activity.  We recommend using startSimpleThread() and/or startProcess() to
    create workers that can be managed by the Command Server, since they are
    compatible with its health monitoring functions.
    """
    def __init__(self,jarvis_handle):
        if jarvis_handle is None:
            from trina.jarvis import Jarvis
            jarvis_handle = Jarvis(self.name())
        self.jarvis = jarvis_handle
        self.status = 'init'
        self._processes = []
        self._threads = []
        self._lock = RLock()

    def name(self):
        """Returns the name used in Jarvis.  Subclass may override this."""
        return self.__class__.__name__

    def healthy(self):
        """Called by CommandServer to check the health of the module.  Optional override.

        Thread safe.
        """
        t = time.time()
        with self._lock:
            for pcess in self._processes:
                if not pcess.is_alive():
                    return False
            for thread,threaddata in self._threads:
                if t - threaddata['last_update'] > THREAD_TIMEOUT:
                    return False
            return True

    def terminate(self):
        """Called by CommandServer to terminate the module.  Optional override
        to close resources.

        Thread safe.
        """
        with self._lock:
            print("Terminating Jarvis module",self.name())
            for proc in self._processes:
                if hasattr(proc,'cleanup_func'):
                    proc.cleanup_func()
                proc.terminate()
                proc.close()
            for i,(thread,threaddata) in enumerate(self._threads):
                if threaddata is not None:
                    threaddata['quit'] = True
                else:
                    try:
                        name = self.name()
                    except NotImplementedError:
                        name = 'Untitled module'
                    print("Warning: Jarvis module %s thread %d created by user, not sure if it can be terminated..."%(name,i))
            threads = self._threads[:]
        if any(thread == threading.current_thread() for thread,threaddata in threads):
            print("  Not joining threads; terminate was called from an existing thread")
        else:
            for thread,threaddata in threads:
                if threaddata is None:
                    print("   Joining custom thread (might hang if the implementer is not careful)")
                else:
                    print("   Joining simple thread",threaddata['name'])
                thread.join()
        print("Termination complete")
        self.status = 'terminated'
        self._processes = []
        self._threads = []

    def startSimpleThread(self,loopfunc,dt,args=(),name=None,initfunc=None,dolock=True):
        """Starts a simple thread that repeatedly calls loopfunc(*args)
        every dt seconds. 

        If dolock = True, this implements a module-wide lock that allows
        loopfunc to access self safely.  However, if loopfunc blocks or has
        long-running calls, then you  may want to set dolock = False and
        implement your own locking mechanisms for better performance.

        Thread safe.
        """
        assert callable(loopfunc),"Need a callable loop callback"
        if name is None:
            name = "Untitled thread"
        
        mod_name = self.name()

        shared_data = {
            'name':name,
            'dt':dt,
            'lock':self._lock,
            'quit':False,
            'dolock':dolock,
            'last_update':time.time()
        }

        def thread_func(shared_data,loop,loopargs):
            if initfunc is not None:
                initfunc(*args)
            looper = TimedLooper(shared_data['dt'],name=mod_name+'.'+shared_data['name'])
            lock = shared_data['lock']
            dolock = shared_data['dolock']
            while looper:
                if dolock:
                    with lock:  
                        #protect against shared self access
                        shared_data['last_update'] = time.time()
                        loop(*loopargs)
                else:
                    shared_data['last_update'] = time.time()
                    loop(*loopargs)
                if shared_data['quit']:
                    #read only, don't need to worry about 
                    break
        thread = Thread(target=thread_func,args=(shared_data,loopfunc,args))
        thread.daemon = True
        thread.start()
        with self._lock:
            self._threads.append((thread,shared_data))
        return thread

    def startCustomThread(self,target,args=()):
        """Starts an unmanaged thread.  You are warned that to properly terminate,
        you will need to overload terminate() and close this thread manually."""
        thread = Thread(target=target,args=args)
        thread.daemon = True
        with self._lock:
            self._threads.append((thread,None))
        return thread

    def startProcess(self,target,args=(),cleanup=None):
        """Starts a new process with the function target, taking arguments args.
        It is assumed that the process will run throughout the life of this module.

        An optional cleanup function cleanup() can be provided.

        Thread safe.
        """
        proc = Process(target=target, args=args)
        proc.daemon = True
        if cleanup is not None:
            proc.cleanup_func = cleanup
        proc.start()
        with self._lock:
            self._processes.append(proc)
        return proc

    def stopThread(self,index_or_name):
        """Safely stops a thread.

        Thread safe.
        """
        print("Stopping Jarvis module",self.name(),"thread",index_or_name)
        with self._lock:
            if isinstance(index_or_name,str):
                index = None
                for i,(thread,threaddata) in enumerate(self._threads):
                    if threaddata['name'] == index_or_name:
                        index = i
                        break
                if index is None:
                    raise ValueError("Invalid thread name "+index_or_name)
            else:
                index = index_or_name
            thread,threaddata = self._threads[index]
            if threaddata is None:
                raise ValueError("Thread %d created by user, can only stop threads started by startSimpleThread")
            threaddata['quit'] = True
            dojoin = not any(thread == threading.current_thread() for thread,threaddata in self._threads)
        if dojoin:
            if threaddata is None:
                print("   Joining custom thread (might hang if the implementer is not careful)")
            else:
                print("   Joining simple thread",threaddata['name'])
            thread.join()
        else:
            print("  Not joining thread %s; terminate was called from an existing thread"%(str(index_or_name)))
        print("Done.")
            

    def stopProcess(self,index):
        """Safely stops a process.

        Thread safe.
        """
        proc = self.processes[index]
        if hasattr(proc,'cleanup_func'):
            proc.cleanup_func()
        proc.terminate()
        proc.close()


