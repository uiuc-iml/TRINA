import os, sys
if(sys.version_info[0] < 3):
    # from future import *
    from imp import reload

else:
    from importlib import reload

from . import ExampleFile
reload(ExampleFile)

from .ExampleFile import Example
