import os,sys

if(sys.version_info[0] < 3):
    # from future import *
    from motion_client import MotionClient
else:
    from motion_client_python3 import MotionClient
    from importlib import reload


from . import test_modules
from . import PointClickNavModule
# import trina_modules.UI

reload(test_modules)
reload(PointClickNavModule)
# UI = reload(trina_modules.UI.UI)

from test_modules import *
from PointClickNavModule import PointClickNav
# from trina_modules.UI.UI import *
