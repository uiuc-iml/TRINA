from threading import Thread, RLock
import threading
from multiprocessing import Process, Manager, Pipe
import time
import sys
import time
from ..utils import TimedLooper

THREAD_TIMEOUT = 30.0

class Module:
    """Base class for Jarvis modules (APIModules and Apps).  Provides some
    convenient functions for starting threads and processes, and monitoring the
    health of those threads.

    Your app should implement name(), and in __init__ it should start the module
    activity.  We recommend using startSimpleThread() and/or startProcess() to
    create workers that can be managed by the Command Server, since they are
    compatible with its health monitoring functions.

    **Runtime responsibilities**

    A Module has the following responsibilities:

    1. In ``__init__``, setup common data and then start one or more threads /
       processes to perform the module's functions. 
    2. Log its health (heartbeat) by periodically calling
       ``jarvis.log_health()``. Or, call ``jarvis.enableHealthChecks(False)``
       on startup to disable health monitoring for this module.
    3. Respect the requests in ``jarvis.getActivityStatus()`` to switch the
       module's activity to 'idle' or 'active' status. 
    4. Monitor the health of its threads in the ``healthy()`` function.
    5. Terminate and release resources when ``terminate()`` is called.

    The Module class provides helpers for items 1, 2, and 3, and automatically
    implements items 4 and 5.

    **Module activity**

    The module activity is controlled by the Command Server, i.e. by UI and
    debugging modules.  To play nice with other modules, a monitoring thread
    should continually check for activity changes using
    ``jarvis.getActivityStatus()``. 

    If switched to 'idle', the module should no longer access jarvis except
    for health logigng and status checking. Your module will have API interfaces
    revoked from jarvis when it is disabled, so be aware that some jarvis.X
    calls may fail during 'idle' status.

    On startup, Command server uses ``self.status`` to determine whether the
    module should start as active or inactive.  By default, Apps are started
    as 'idle' while APIModules are started as 'active', but you can override
    this.

    Attributes:
        jarvis (jarvis.Jarvis): the Jarvis handle. If None is provided, a 
            default (temp) module is assumed, which has no access to APIs 
            that use shared memory.
        status (str): typically either 'idle' or 'active'.  Upon construction,
            this tells CommandServer whether the module wishes to be 
            considered active or idle on startup.
    """
    def __init__(self, Jarvis, Verbose = 0):
        if Jarvis is None:
            from trina import jarvis
            Jarvis = jarvis.Jarvis(self.name())
        self.jarvis = Jarvis
        self.status = 'idle'
        self.verbose = Verbose
        self._processes = []
        self._threads = []
        self._lock = RLock()

    @classmethod
    def name(cls):
        """Returns the identifier used to refer to this module in Jarvis. 
        Default is the class' name.  Optional override.
        """
        return cls.__name__

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
                if threaddata is None:
                    #not a managed thread
                    continue
                if threaddata['exception'] is not None:
                    if self.verbose:
                        print("{}.healthy(): exception was raised, returning False".format(self.name()))
                    return False
                if t - threaddata['last_update'] > THREAD_TIMEOUT:
                    if self.verbose:
                        print("{}.healthy(): thread's last update was {}s ago, returning False".format(self.name(),t - threaddata['last_update']))
                    return False
            return True

    def terminate(self):
        """Called by CommandServer to terminate the module.  Optional override
        to close resources.

        Thread safe.
        """
        with self._lock:
            if self.verbose:
                print("Terminating Jarvis module",self.name())
            for proc in self._processes:
                if proc is None:
                    continue
                if hasattr(proc,'cleanup_func'):
                    proc.cleanup_func()
                proc.terminate()
            for i,(thread,threaddata) in enumerate(self._threads):
                if threaddata is not None:
                    threaddata['quit'] = True
                else:
                    try:
                        name = self.name()
                    except NotImplementedError:
                        name = 'Untitled module'
                    print("%s.terminate(): Warning, thread %d created by user, not sure if it can be terminated..."%(name,i))
            threads = self._threads[:]
        if any(thread == threading.current_thread() for thread,threaddata in threads):
            if self.verbose:
                print("  Not joining threads; terminate was called from an existing thread")
        else:
            for thread,threaddata in threads:
                if threaddata is None:
                    if self.verbose:
                        print("   Joining custom thread (might hang if the implementer is not careful)")
                else:
                    if self.verbose:
                        print("   Joining thread",threaddata['name'])
                thread.join()
        if self.verbose:
            print("Termination complete")
        self.status = 'terminated'
        self._processes = []
        self._threads = []

    def startSimpleThread(self,loopfunc,dt,args=(),name=None,initfunc=None,dolock=True):
        """Starts a simple thread that repeatedly calls ``loopfunc(*args)``
        every ``dt`` seconds. 

        Args:
            loopfunc (callable): the function called every loop
            dt (float): the frequency of the loop.
            args (tuple, optional): the arguments passed to ``loopfunc``
            name (str, optional): an identifier that can be used to refer to
                the thread
            initfunc (callable, optional): if given, this is called within the
                created thread.
            dolock (bool, optional): if True (default), this uses a module-wide
                lock (``self._lock``) that allows ``loopfunc`` to access
                ``self`` safely.
        
        The default ``dolock=True`` is the safest option.  However, if
        ``loopfunc`` blocks or has long-running calls, then you may want to set
        ``dolock = False`` and implement your own locking mechanisms for better
        performance.

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
            'last_update':time.time(),
            'exception':None
        }

        def thread_func(shared_data,loop,loopargs):
            if initfunc is not None:
                try:
                    initfunc(*args)
                except Exception as e:
                    shared_data['exception'] = e
                    if self.verbose:
                        print("Exception {} occurred in module {} thread {} init func".format(str(e),mod_name,name))
                        import traceback
                        traceback.print_exc()
                    return
            looper = TimedLooper(shared_data['dt'],name=mod_name+'.'+shared_data['name'],warning_frequency=('auto' if self.verbose > 0 else 0))
            lock = shared_data['lock']
            dolock = shared_data['dolock']
            while looper:
                if dolock:
                    with lock:  
                        #protect against shared self access
                        shared_data['last_update'] = time.time()
                        try:
                            loop(*loopargs)
                        except Exception as e:
                            shared_data['exception'] = e
                            if self.verbose:
                                print("Exception {} occurred in module {} thread {}".format(str(e),mod_name,name))
                                import traceback
                                traceback.print_exc()
                            break
                else:
                    shared_data['last_update'] = time.time()
                    try:
                        loop(*loopargs)
                    except Exception as e:
                        shared_data['exception'] = e
                        if self.verbose:
                            print("Exception {} occurred in module {} thread {}".format(str(e),mod_name,name))
                            import traceback
                            traceback.print_exc()
                        break
                if shared_data['quit']:
                    #read only, don't need to worry about 
                    break
        thread = Thread(target=thread_func,args=(shared_data,loopfunc,args))
        thread.daemon = True
        with self._lock:
            thread.start()
            self._threads.append((thread,shared_data))
        return thread

    def startCustomThread(self,target,args=()):
        """Starts an unmanaged thread.  You are warned that to properly terminate,
        you will need to overload terminate() and close this thread manually."""
        thread = Thread(target=target,args=args)
        thread.daemon = True
        with self._lock:
            self._threads.append((thread,None))
        thread.start()
        return thread

    def startMonitorThread(self,dt=0.1,name="monitor",dolock=True,onLoop=None,onActivate=None,onDeactivate=None):
        """Starts a thread to update the module's health and status. This 
        handles heartbeats and activity status updates.

        The dt, name, and dolock arguments are the same as in
        :func:`startSimpleThread`.

        To add additional calls per loop, provide a function as onLoop.

        To call a funciton when the module is activated / deactivated, provide
        onActivate / onDeactivate.
        """
        def _infoLoop(self):
            self.jarvis.log_health()
            status = self.jarvis.getActivityStatus()

            if status == 'active':
                if self.status == 'idle':
                    if self.verbose:
                        print('\n\n\n Activating Module {} \n\n\n'.format(self.name()))
                    self.status = 'active'
                    if onActivate is not None:
                        onActivate()
                if onLoop is not None:
                    onLoop()

            elif status == 'idle':
                if self.status == 'active':
                    if self.verbose:
                        print('\n\n\n Deactivating Module {} \n\n\n'.format(self.name()))
                    self.status = 'idle'
                    if onDeactivate is not None:
                        onDeactivate()

            #This is new -- added so that redisRpc calls are handled
            self.processRedisRpcs()
        return self.startSimpleThread(_infoLoop,dt,args=(self,),name=name,dolock=dolock)


    def startProcess(self,target,args=(),cleanup=None):
        """Starts a new process with the function ``target``, taking arguments
        ``args``. It is assumed that the process will run throughout the life
        of this module.

        An optional cleanup function ``cleanup(*args)`` can be provided. 

        .. note::
            The target function cannot communicate to the calling thread or any
            other process through the "self" object. Instead, the class
            constructor must use some inter-process communication primitives
            in the ``multiprocessing`` module.

        Thread safe.
        """
        proc = Process(target=target, args=args)
        proc.daemon = True
        if cleanup is not None:
            proc.cleanup_func = lambda : cleanup(*args)
        with self._lock:
            #disable access to any APIs that are not compatible with IPC
            interprocess_apis = dict()
            for k,a in self.jarvis.apis.items():
                if a.interprocess():
                    interprocess_apis[k] = a
            sanitized_jarvis = Jarvis(self.jarvis.name,interprocess_apis,self.jarvis.server)
            orig_jarvis = self.jarvis
            self.jarvis = sanitized_jarvis
            proc.start()  #this forks with a copy of self.jarvis
            self.jarvis = orig_jarvis
            self._processes.append(proc)
        return proc

    def stopThread(self,index_or_name):
        """Safely stops a thread.

        Thread safe.
        """
        if self.verbose:
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
                if self.verbose:
                    print("   Joining custom thread (might hang if the implementer is not careful)")
            else:
                if self.verbose:
                    print("   Joining thread",threaddata['name'])
            thread.join()
        else:
            if self.verbose:
                print("  Not joining thread %s; terminate was called from an existing thread"%(str(index_or_name)))
        if self.verbose:
            print("Done.")

    def stopProcess(self,index):
        """Safely stops a process.

        Thread safe.
        """
        proc = self.processes[index]
        if proc is None:
            if self.verbose:
                print("Process",index,"already terminated")
            return
        if hasattr(proc,'cleanup_func'):
            proc.cleanup_func()
        proc.terminate()
        self.processes[index] = None


