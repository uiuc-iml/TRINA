import reem
from packaging import version
if version.parse(reem.__version__) < version.parse('0.1.0'):
    raise ImportError("Require reem version >= 0.1.0, try reinstalling from source")
from reem import KeyValueStore
from threading import Lock
import redis
try:
    from . import settings
except SystemError: #must have been run as a script
    import sys
    import os
    sys.path.append(os.path.expanduser("~/TRINA"))
    from trina import settings

class StateServer(KeyValueStore):
    """StateServer should usually be treated just like a normal
    REEM KeyValueStore object.

    Overall, the use of StateServer is preferable to ``reem.KeyValueStore``
    because it's:

    - Pickleable
    - Extra convenience functions (:func:`redis_client`)

    Using the :func:`var`, :func:`set`, :func:`get` methods have following
    advantages over using the [] syntax defined in REEM:

    - Can accept paths in the form of string paths ``['key1','key2']`` or 
      period- separated keys ``'key1.key2'``
    - More tolerant of uninitialized keys and will fill in empty dicts as
      needed.

    """
    def __init__(self,host=None):
        if host is None:
            host = settings.redis_server_ip()
        if isinstance(host,StateServer):
            host = host.host
        self.host = host
        KeyValueStore.__init__(self,host)

    def __repr__(self):
        return self.host

    def __str__(self):
        return "StateServer(%s)"%(self.host,)

    def __getstate__(self):
        return self.host

    def __setstate__(self,host):
        self.host = host
        interface = RedisInterface(host=host)
        interface.initialize()
        KeyValueStore.__init__(self,interface)

    def redis_client(self):
        """Returns a raw handle to the redis client."""
        return self.interface.client
    
    def var(self,path):
        """Given a path (period-separated str or list of str), returns a 
        :class:`StateServerVar` object that accesses the key using get()/set().
        """
        return StateServerVar(self,path)

    def get(self,path):
        """Given a path (period-separated str or list of str), returns a JSON
        object or numpy array at path by recursively descending into self.
        """
        if isinstance(path,str):
            res = path.split('.',1)
            if len(res) == 1:
                res = [res[0],'']
            return self.redis_client().jsonget(res[0],'.'+res[1])
        else:
            assert isinstance(path,(list,tuple))
            assert len(path) > 0    
        res = self[path[0]]
        for p in path[1:]:
            res = res[p]
        return res.read()

    def set(self,path,item):
        """Given a path (list of str), sets a value in the state
        server.  The item must be json-encodable or a numpy array.

        If you are accessing a path with multiple subkeys that don't
        exist, the subkeys will be added.
        """
        if isinstance(path,str):
            if item.__class__.__name__ == 'ndarray':
                path = path.split('.')
            else:
                res = path.split('.',1)
                if len(res) == 1:
                    res = [res[0],'']
                try:
                    self.redis_client().jsonset(res[0],'.'+res[1],item)
                    return
                except redis.exceptions.ResponseError:
                    #need to fall into slower method below
                    self.set_new(path.split('.'),item)
                    return
        else:
            assert isinstance(path,(list,tuple))
            assert len(path) > 0
        if len(path) == 1:
            self[path[0]] = item
        else:
            res = self[path[0]]
            for i,p in enumerate(path[1:-1]):
                res = res[p]
            try:
                res[path[-1]] = item
            except redis.exceptions.ResponseError:
                #missing subkey somewhere along the line
                self.set_new(path,item)

    def set_new(self,path,item):
        """Creates a new item if the path doesn't exist yet"""
        if isinstance(path,str):
            path = path.split('.')
                
        def tryassign(node,path_remaining,path_to_me,val):
            key = path_remaining[0]
            if len(path_remaining) == 1:
                try:
                    node[key] = val
                    return node,val
                except redis.exceptions.ResponseError:
                    return None,{key:val}
            assigned,subval = tryassign(node[key],path_remaining[1:],path_to_me+[key],val)
            if assigned is not None:
                return assigned,subval
            #could not assign to children, try assigning to me
            try:
                node[key] = subval
                return node,subval
            except redis.exceptions.ResponseError:
                return None,{key:subval}

        node,val = tryassign(self,path,[],item)
        return node,val


class StateServerVar:
    def __init__(self,server,path):
        """This will create all parent keys in path upon a failed set"""
        self.server = server
        if isinstance(path,str):
            path = path.split('.')
        else:
            assert isinstance(path,(list,tuple))
            assert len(path) > 0
        self.path = path
        res = server
        for p in path:
            prev = res
            res = res[p]
            try:
                res.type()
            except Exception:
                if isinstance(p,int):
                    raise KeyError("Invalid index into array {}".format(path))
                #key doesn't exist
                prev.write(dict())
                res = res[p]
        self.node = res[self.key]
    def get(self):
        """Reads the value at the given key from the state server"""
        return self.node.read()
    def set(self,value):
        """Sets the value at the given key into the state server"""
        return self.node.write(value)


if __name__ == '__main__':
    print("StateServer testing")
    s = StateServer()
    try:
        res = s.get("example")
        print("example:",res)
    except Exception as e:
        print(e)
        print("(correct) Empty return value under 'example' key")
    s.set("example",{'a':{'foo':3,'bar':[6,8]},'b':5})
    res = s.get(["example",'a'])
    print("[example][a]",res)
    res = s.get("example.a")
    print("example.a",res)
    res = s.get("example.a.bar")
    print("example.a.bar",res)
    svar = s.var('example.a.foo')
    svar.set(1234)
    print("example.a.foo",svar.get()," = 1234")
    s.set("example.c.foo",12345)
    print("example.c",s.get("example.c")," = 12345")