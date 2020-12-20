import json
import os
import weakref

class NestedJsonAccessor:
    """Makes it really easy to define a JSON structure distributed across
    files.  With this structure, JSON files can refer to other files using
    values of the form "@FILENAME.JSON" and they will be automatically
    loaded when accessed.

    Almost all dict/list accessors are implemented.  To treat this like a plain
    dict, call asdict().  By default, this will only return the contents of the
    immediate JSON file. To get the whole structure. call asdict(True).
    To treat a list object as a plain list, use aslist() or list(self).

    To save back to disk, call the save() function.  This will preserve the 
    data and file structure as much as possible.
    """
    def __init__(self,val=None,parent=None,key=None):
        self.val=val
        self.fn = None if parent is None else parent.fn
        self.file = None if parent is None else parent.file
        self.mydir = None if parent is None else parent.mydir
        self.valueCache = dict()
        self.mykey = key
        self.parent = parent
        self.dirty = False
    def load(self,fn):
        with open(fn,'r') as f:
            try:
                self.val = json.load(f)
            except json.decoder.JSONDecodeError as e:
                print("Error parsing JSON file",fn)
                raise
        self.fn = fn
        self.file = _NestedJsonAccessorFile(fn)
        self.mydir = os.path.dirname(fn)
    def save(self,fn=None):
        self.file.save(self)
    def __str__(self):
        self._compress()
        return str(self.val)
    def __repr__(self):
        self._compress()
        return repr(self.val)
    def __len__(self):
        return len(self.val)
    def __contains__(self,key):
        return key in self.val
    def __iter__(self):
        self._compress()
        return iter(self.val)
    def keys(self):
        return self.val.keys()
    def values(self):
        raise NotImplementedError("Can't do values() yet")
    def items(self):
        raise NotImplementedError("Can't do items() yet")
    def append(self,v):
        return self.val.append(v)
    def aslist(self):
        if not isinstance(self.val,list):
            raise ValueError("Value is not a list")
        self._compress()
        return self.val
    def asdict(self,full=False):
        if not isinstance(self.val,dict):
            raise ValueError("Value is not a dictionary")
        if full:
            if len(self.valueCache)==0:
                return self.val
            res = self.val.copy()
            for k,v in self.valueCache.items():
                res[k] = v
            return res
        else:
            self._compress()
            return self.val
    def _compress(self):
        for k,v in self.valueCache.items():
            if v.file is self.file and v.dirty:
                self.val[k] = v._compress()
        return self.val
    def setref(self,key,fn):
        """Indicates that key should be saved to file fn and replaced with a
        reference.  fn may be a relative or absolute reference.
        """
        raise NotImplementedError("TODO: save references")
    def __setitem__(self,key,value):
        if isinstance(value,NestedJsonAccessor):
            value._compress()
            return self.__setitem__(key,value.val)
        if key in self.valueCache:
            del self.valueCache
        self.val[key] = value
        self.dirty = True
        if self.file is not None:
            self.file.dirty = True
    def __getitem__(self,key):
        if key in self.valueCache:
            return self.valueCache[key]
        try:
            value = self.val[key]
        except KeyError:
            raise KeyError("Invalid key {} for item {} in file {}".format(key,('root' if self.mykey is None else self.mykey),self.fn))
        if isinstance(value,str) and value.startswith('@'):
            base,ext = os.path.splitext(value)
            if ext == '.json':
                res = NestedJsonAccessor(parent=weakref.proxy(self),key=key)
                fn = value[1:]
                if os.path.isabs(fn):
                    res.load(fn)
                else:
                    res.load(os.path.join(self.mydir,fn))
                self.file.children.append(res)
                res.file.parent = weakref.proxy(self.file)
                self.valueCache[key] = res
                return res
            elif ext in NestedJSONAccessor._decoders:
                return NestedJSONAccessor._decoders(value[1:])
            return value
        else:
            if isinstance(value,(dict,list)):
                #test whether it needs an accessor
                res = NestedJsonAccessor(value,parent=weakref.proxy(self),key=key)
                self.valueCache[key] = res
                return res
            return value
    def get(self,key,default):
        if key in self.valueCache:
            return self.valueCache[key]
        if not isinstance(self.val,dict):
            raise AttributeError("Can't call get on a NestedJSONAccessor(list), file",self.fn)
        value = self.val.get(key,default)
        if isinstance(value,str) and value.startswith('@'):
            base,ext = os.path.splitext(value)
            if ext == '.json':
                res = NestedJsonAccessor(parent=weakref.proxy(self),key=key)
                fn = value[1:]
                if os.path.isabs(fn):
                    res.load(fn)
                else:
                    res.load(os.path.join(self.mydir,fn))
                self.file.children.append(res)
                res.file.parent = weakref.proxy(self.file)
                self.valueCache[key] = res
                return res
            elif ext in NestedJsonAccessor._decoders:
                return NestedJSONAccessor._decoders(value[1:])
            return value
        else:
            if isinstance(value,(dict,list)):
                res = NestedJsonAccessor(value,parent=weakref.proxy(self),key=key)
                self.valueCache[key] = res
                return res
            return value

    @classmethod
    def registerCodec(cls,ext,encoder,decoder):
        if not ext.startswith('.'):
            raise ValueError("ext must be a file extension starting with .")
        if not callable(encoder):
            raise ValueError("encoder must be a 1-argument callable function")
        if not callable(decoder):
            raise ValueError("decoder must be a 1-argument callable function")
        cls._encoders[ext] = encoder
        cls._decoders[ext] = decoder

    _decoders = dict()
    _encoders = dict()

class _NestedJsonAccessorFile:
    def __init__(self,fn):
        self.fn = fn
        self.dirty = False
        self.parent = None
        self.children = []
    def save(self,accessor,fn=None):
        val = self.compress(accessor)
        with open(self.fn,'w') as f:
            json.dump(val,f)
        if fn is None:
            if not self.dirty:
                print("NestedJSONAccessor.save(): Not saving JSON file",self.fn,"because it has not been changed")
                return
            for acc in self.children:
                if acc.file.dirty:
                    acc.file.save(acc)
        else:
            for acc in self.children:
                if acc.file.dirty:
                    print("NestedJSONAccessor.save(): Warning, item",acc.mykey,"is dirty but will not be saved since we are saving to a different file than originally opened")
            return
    def compress(self,accessor):
        if accessor.parent is not None and accessor.parent.fn == self.fn:
            if accessor.dirty:
                accessor.parent.val[accessor.key] = accessor.val
            return self.compress(accessor.parent)
        return accessor.val

