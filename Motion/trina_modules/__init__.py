import os,sys

if(sys.version_info[0] < 3):
    # from future import *
    from motion_client import MotionClient
else:
    from motion_client_python3 import MotionClient
    from importlib import reload

# from importlib import reload

path = os.path.expanduser('~/TRINA/Motion/trina_modules/')
sys.path.append(path)


import trina_modules.test1
import trina_modules.test2
# import trina_modules.UI

test1 = reload(test1)
test2 = reload(test2)
# UI = reload(trina_modules.UI.UI)

from trina_modules.test1 import *
from trina_modules.test2 import *
# from trina_modules.UI.UI import *
