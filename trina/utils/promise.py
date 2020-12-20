import time

class PromiseTimeout(Exception):
    def __init__(self,promise):
        self.promise = promise
    def __str__(self):
        if self.promise._name is not None:
            return "PromiseTimeout %s timed out"%(self.promise._name,)
        return "PromiseTimeout timed out"


class Promise:
    """A placeholder for an asynchronous result, inspired by the twisted
    package. Can wait passively by testing
    
        if promise:
            val = promise.value()

    setting a callback function fn(value) with 

        promise.setCallback(fn)

    or blocking until it arrives using:

        val = promise.await()

    If you wish to put a timeout on await, use `val = promise.await(timeout)`.

    value() will raise a RuntimeError if the value is not available yet.

    await(timeout) will raise a PromiseTimeout exception if the timeout is
    reached without the result arriving.

    For functions writing to this value: use callback(value) to set the value
    when it is available.
    """
    def __init__(self,name=None):
        self._name = name
        self._read = False
        self._value = None
        self._error = None
        self._callback = None
        self._errback = None

    def setCallback(self,fn):
        assert callable(fn),"fn nees to be a callable object"
        self._callback = fn
    
    def setErrback(self,fn):
        assert callable(fn),"fn nees to be a callable object"
        self._errback = fn

    def __nonzero__(self):
        return self.available()
    
    def __bool__(self):
        return self.available()

    def available(self):
        return self._read

    def error(self):
        return self._error
    
    def value(self):
        if self._read:
            if self._error is not None:
                raise RuntimeError(self._error+" occurred")
            return self._value
        raise RuntimeError("Value is not available")

    def await(self,timeout=None,resolution=0.001):
        """Blocks until the response is available.  If you wish to put a timeout
        on this call, use the timeout argument.  For finer control on the polling
        frequency, change the resolution argument.
        """
        t0 = time.time()
        while not self.available():
            time.sleep(resolution)
            t = time.time()
            if timeout is not None and t - t0 > timeout:
                raise PromiseTimeout(self)
        if self.error():
            raise RuntimeError(self.error())
        return self.value()

    def callback(self,value):
        """Called by the provider to set the value."""
        assert not callable(value),"callback() is meant to be used by the caller to set the promise's value. Do you mean to use setCallback()?"
        if self._read:
            raise RuntimeError("Can't call callback() twice on a Promise object")
        self._value = value
        self._read = True
        if self._callback is not None:
            self._callback(value)

    def errback(self,error):
        """Called by the provider to indicate an error."""
        assert not callable(error),"errback() is meant to be used by the caller to set the promise's error condition. Do you mean to use setErrback()?"
        if self._read:
            raise RuntimeError("Can't call errback()/callback() twice on a Promise object")
        self._error = error
        self._read = True
        if self._errback is not None:
            self._errback(error)
