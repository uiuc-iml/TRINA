import os,sys

if(sys.version_info[0] < 3):
    # from future import *
    pass
else:
    from importlib import reload


from . import test_modules
from . import PointClickNavModule
from . import DirectTeleOperationModule
from . import LoggerModule
# import trina_modules.UI

reload(test_modules)
reload(PointClickNavModule)
reload(DirectTeleOperationModule)
reload(LoggerModule)
# UI = reload(trina_modules.UI.UI)

from test_modules import *
from PointClickNavModule import PointClickNav
from DirectTeleOperationModule import DirectTeleOperation
from LoggerModule import StateLogger
# from trina_modules.UI.UI import *
