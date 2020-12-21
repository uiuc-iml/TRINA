import uuid
from threading import Thread
import json
import time
import sys
from .module import Module


class APILayer:
    """An APILayer is an abstract base class that should provide a frontend 
    for an APIModule for other modules to use.  These methods may interact 
    with the APIModule in various ways, as designed by the implementer of the
    APIModule.

    The communication model is multi-paradigm.  Get/set, RPC, direct object
    access, shared memory, and other implementations are possible.

    For users:

    - You will need to read the implementer's documentation of the methods that
      they choose to expose for their API.

    For API implementers:

    - Your subclass should implement methods and clear docstrings for each
      available method.
    - In each of your methods, it is recommended that you implement your API 
      using :func:`_redisGet`, :func:`_redisSet`, :func:`_redisRpc`,
      :func:`_redisRpcNoReply`, or :func:`_moduleCommand`. 

      * :func:`_redisGet` retrieves a key from the Redis database.  (In your
        API-module, you will probably set the same key for users to read.)
      * :func:`_redisSet` sets a key from the Redis database.  (In your API-
        module, you will probably read from this key to see what users have
        set.)
      * :func:`_redisRpc` uses an RPC mechanism to allow other users to call 
        functions in your module.  By default, if you implement a method
        ``foo`` in your module and your API implements ``foo(self,*args)``
        using ``return self._redisRpc("foo",*args)``, then RPC will call your
        module's ``foo`` and return the result to the caller.
      * :func:`_redisRpcNoReply` is the same as :func:`_redisRpc` but does not
        request a return value. 
      * :func:`_moduleCommand` uses a lightweight, one-way RPC similar to
        _redisRpcNoReply. It currently is used to communicate with the Command
        Server and Motion Server, but may be deprecated in the future.

    - For performance, you can pass shared objects to your API subclass.
      However, you must be careful with making accesses thread-safe!

    All arguments to these functions should have types compatible with JSON
    (bool, int, float, str, list, and dict).  _redisGet / _redisSet can also
    accept numpy arrays.  If you wish to allow your API to accept / return
    other datatypes, you must do the conversion.

    Note that if you need to respond to sets/gets differently depending on the
    other module making the call, use ``self._caller_name`` in the path. 

    RPC calls typically return a :class:`RpcPromise` object, which the caller can
    poll using :func:`RpcPromise.available` or wait on using
    RpcPromise.wait().  If you really wish to have your API call block
    on an RPC call, your implementation can end with
    `return self._redisRpc("foo",*args).wait()`.  Note that this is dangerous
    in case your module hangs or goes down.

    .. automethod:: _redisVar
    .. automethod:: _redisGet
    .. automethod:: _redisSet
    .. automethod:: _redisRpc
    .. automethod:: _redisRpcNoReply
    .. automethod:: _moduleCommand
    """
    def __init__(self,api_name,caller_name,state_server=None):
        self._api_name = api_name
        self._caller_name = caller_name
        self._state_server = state_server
        self._redis_client = state_server.redis_client()

    def _redisVar(self,path):
        """Returns an object that you can use as a get-set variable. Typically
        used in __init__ by calling:

            self.[ITEM_NAME] = self._redisVar(['PATH','TO','OBJECT'])

        and then users can call self.[ITEM_NAME].set(value) and
        self.[ITEM_NAME].get(value).

        Slightly faster than implementing API calls as separate _redisGet /
        _redisSet pairs.
        """
        return self._state_server.var(path)

    def _redisGet(self,path):
        """Uses the Redis server to retrieve a value under one or more keys given
        in path.
        """
        return self._state_server.get(path)

    def _redisSet(self,path,item):
        """Uses the Redis server to set a value from path.
        """
        return self._state_server.set(path,item)

    def _redisRpc(self,fn,*args,**kwargs):
        """Uses the Redis server to perform an RPC call.

        Returns:
             RpcPromise
        """
        id = str('$' + uuid.uuid1().hex)
        self._state_server.set_new([self._api_name,'RPC_FEEDBACK',id],{'REPLIED': False, 'MSG': None})
        rpc_node = self._state_server[self._api_name]['RPC_FEEDBACK']
        msg = {'fn': fn, 'args': args, 'kwargs':kwargs, 'id': id, 'from':self._caller_name}
        self._redis_client.rpush(self._api_name+"_RPC_QUEUE",json.dumps(msg))
        return RpcPromise(rpc_node,self._api_name,fn,id)

    def _redisRpcNoReply(self,fn,*args,**kwargs):
        """Uses the Redis server to perform an RPC call with no reply.

        Returns:
            None.
        """
        msg = {'fn': fn, 'args': args, 'kwargs':kwargs,'id': None, 'from':self._caller_name}
        self._redis_client.rpush(self._api_name+"_RPC_QUEUE",json.dumps(msg))

    def _moduleCommand(self,fn,*args):
        """Calls a function fn implemented in the module, specifically a
        function defined in the result of
        :func:`APIModule.moduleCommandObject`. ``fn(*args)`` is expected to 
        have no return value.
        """
        final_string = '{}.{}({})'.format(self._api_name,fn,','.join(json.dumps(a) for a in args))
        self._redis_client.rpush(self._caller_name+'_MODULE_COMMANDS',final_string)
        print('sending ',final_string)



class APIModule(Module):
    """Base class for  modules that implement an API that can be used by 
    other modules (apps and API-modules) through the Jarvis handle. 

    The API is exposed using the :func:`api` call.  This produces a APILayer
    interface that acts as the "frontend" for your module under other modules'
    jarvis.X objects, where X= the result from :func:`apiName`. 

    The API interface must access shared resources in a thread-safe manner.
    The easiest way to do this is to use the Redis server as an intermediary. 

    Performance-oriented modules (e.g., sensing, perception) may prefer to
    use shared memory access.  However, your API must be carefully designed
    with locks, etc. in order to avoid clashes.

    Your API can use the Redis server RPC functionality to safely call
    functions in this module.  A basic RPC server would start a thread to call
    self.processRedisRpcs in the __init__ function, like so::

        class MathExternalAPI(Jarvis.APILayer):
            def add(self,a,b):
                #add two numbers
                self._redisRpc("add",a,b)

            def foo(self,a,b):
                #do something mysterious
                self._redisRpc("foo",a,b)

        class MathAPIModule(Jarvis.APIModule):
            def __init__(self,jarvis):
                Jarvis.APIModule.__init__(self,jarvis)
                rate = 1.0/50.0   #serve at 50Hz ... 
                self.startSimpleThread(self.processRedisRpcs,rate)

            def add(self,a,b):
                return a+b

            def foo(self,a,b):
                return a**2+3*b*a

            def api(self,other_module_name,**kwargs):
                return SumExternalAPI(other_module_name,**kwargs)

    Note that it is better practice to set the rate dynamically, something like
    ``settings.get('MathModule.rate')``.
    """
    def __init__(self,Jarvis, Verbose=0):
        Module.__init__(self,Jarvis,Verbose)
        self.status = 'active'

    def apiName(self):
        """Returns an identifier that will be used to refer to the API under
        jarvis and the state server."""
        return self.name()

    def api(self,other_module_name,**jarvis_handles):
        """To implement an API module, subclass must return a :class:`APILayer`
        subclass that implements the communication from the other module to
        your module.

        Typical implementation is::

            return MyJarvisAPI(self.apiName(),other_module_name,**jarvis_handles)

        If you want to restrict which modules have access to your module, you
        can raise a NotImplementedError() if the other module should be denied
        access.
        """
        raise NotImplementedError()

    def moduleCommandObject(self):
        """If the APILayer returned by :func:`api` implements functions using
        :func:`APILayer._moduleCommand`, return a pair ``(api_name,object)``
        object whose methods will be called. 

        Note: the object's methods will be called from an arbitrary non-module
        thread, so make sure the methods are thread-safe.
        """
        raise NotImplementedError()

    def startMonitorAndRpcThread(self,dt=0.01,name="monitor+rpc",dolock=True,onLoop=None,onActivate=None,onDeactivate=None):
        """Same functions as the normal monitor thread, but also calls
        :func:`processRedisRpcs`.
        """
        oldOnLoop = onLoop
        def newOnLoop(self_=self,oldfunc=oldOnLoop):
            if oldfunc is not None:
                oldfunc()
            self_.processRedisRpcs()
        onLoop = newOnLoop
        Module.startMonitorThread(self,dt,name,dolock,onLoop,onActivate,onDeactivate)

    def doRpc(self,fn,args,kwargs):
        """To implement an API module that implements RPC calls, subclass may 
        override this to process an RPC call.  Default implementation looks for
        ``fn`` in ``self``, and executes the arguments as interpreted as python
        strings.
        """
        if hasattr(self,fn):
            return getattr(self,fn)(*args,**kwargs)
            #return getattr(self,fn)(*args)
        raise NotImplementedError("RPC call "+fn+" is not implemented")

    def processRedisRpcs(self):
        """If you are implementing an API module that implements _redisRpc
        calls, your main module must this periodically to process redis-RPC
        calls.
        """
        redis_client = self.jarvis._redis_client
        key = self.apiName() + "_RPC_QUEUE";
        rpc_node = self.jarvis._state_server[self.apiName()]['RPC_FEEDBACK']
        with redis_client.pipeline() as pipe:
            n = redis_client.llen(key)
            for i in range(n):
                pipe.lpop(key)
            res = pipe.execute()
        for msg in res:
            msg = json.loads(msg)
            try:
                res = self.doRpc(msg['fn'],msg['args'],msg['kwargs'])
            except Exception as e:
                print("Exception while executing",msg['fn'],"with args",msg['args'],msg['kwargs'])
                import traceback
                traceback.print_exc()
                res = "__REDIS_RPC_ERROR__"    
            if msg['id'] is not None:
                try:
                    rpc_node[msg['id']] = {'MSG':res,'REPLIED':True}
                except Exception:
                    print("API %s warning: error responding to RPC call %s"%(self.apiName(),msg['id']))
                    pass

    def getRedisRpc(self):
        """Manual alternative to :func:`processRedisRpcs`: you can periodically
        call this to get individual RPC requests.  This is useful for long-
        running RPC calls where you do not want to block your module.

        Returns:
            dict or None: None if there are no pending RPC requests.  
            Otherwise, returns a dict ``request`` with fields "fn", "args",
            "kwargs", and "id". It will also have "from" indicating which
            module it's coming from, but this can be ignored.

        Usage is::

            request = self.getRedisRpc()
            res = self.doRpc(request['fn'],request[args],request[kwargs])
            if request['id'] is not None:
                self.setRedisRpc(request['id'],res)

        Once you handle the request, you must pass the contents of
        ``request['id']`` back to :func:`setRedisRpc` as its first argument, if it
        is not None.
        """
        redis_client = self.jarvis._redis_client
        key = self.apiName() + "_RPC_QUEUE";
        n = redis_client.llen(key)
        if n > 0:
            return json.loads(pipe.lpop(key))
        return None

    def setRedisRPC(self,id,reply):
        """Manually replies to a previous request from :func:`getRedisRpc`."""
        if id is None:
            return
        rpc_node = self.jarvis._state_server[self.apiName()]['RPC_FEEDBACK']
        #rpc_node[msg['id']] = {'MSG':res,'REPLIED':True}
        try:
            rpc_node[msg['id']] = {'MSG':res,'REPLIED':True}
        except Exception:
            #maybe this was a stale request from a prior Ctrl+C
            print("API %s warning: error responding to RPC call %s"%(self.apiName(),msg['id']))
            pass




class RpcPromiseTimeout(Exception):
    def __init__(self,promise):
        self.promise = promise
    def __str__(self):
        return "jarvis.RpcPromise %s.%s %s timed out"%(promise.api,promise.fn,promise.id)

class RpcPromise:
    """A placeholder for an RPC result.  Can wait passively by testing
    
        if promise:
            val = promise.value()

    or blocking until it arrives using:

        val = promise.wait()

    If you wish to put a timeout on wait, use `val = promise.wait(timeout)`.

    value() will raise a RuntimeError if the value is not available yet.

    wait(timeout) will raise a RpcPromiseTimeout exception if the
    timeout is reached without a reply.
    """
    def __init__(self,server_key,api,fn,id):
        self.server_key = server_key
        self.api = api
        self.fn = fn
        self.id = id
        self._read = False
        self._value = None
    
    def __nonzero__(self):
        return self.available()
    
    def __bool__(self):
        return self.available()
    
    def available(self):
        return self.server_key[self.id]['REPLIED'].read()
    
    def value(self):
        if self._read:
            return self._value
        res = self.server_key[self.id].read()
        if not res['REPLIED']:
            raise RuntimeError("Value is not available")
        if 'ERROR' in res:
            raise IOError("jarvis.RpcPromise errored out with message "+res['ERROR'])
        self._read = True
        self._value = res['MSG']
        """
        #TODO: delete from reply queue, making sure this is done atomically
        all_replies = self.server_key.read()
        del all_replies[self.id]
        self.server_key = all_replies
        """
        return self._value

    def wait(self,timeout=None,resolution=0.001):
        """Blocks until the response is available.  If you wish to put a timeout
        on this call, use the timeout argument.  For finer control on the polling
        frequency, change the resolution argument.
        """
        t0 = time.time()
        while not self.available():
            time.sleep(resolution)
            t = time.time()
            if timeout is not None and t - t0 > timeout:
                raise RpcPromiseTimeout(self)
        return self.value()
