import os,sys

if(sys.version_info[0] < 3):
    # from future import *
    from imp import reload
else:
    from importlib import reload

# from importlib import reload

# path = os.path.expanduser('~/TRINA/Motion/trina_modules/test_modules')
# sys.path.append(path)

from . import test1, test2
# import trina_modules.UI

test1 = reload(test1)
test2 = reload(test2)

from .test1 import *
from .test2 import *
