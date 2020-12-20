
def lock_wrap(lock,fn):
    """Wraps a function fn with a lock acquisition.  Returns a callable
    that accepts the same signature as fn."""
    assert callable(fn)
    def lockfunc(*args,**kwargs):
        with lock:
            return fn(*args,**kwargs)
    return lockfunc
