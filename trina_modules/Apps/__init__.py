import os,sys
import traceback

if(sys.version_info[0] < 3):
    # from future import *
    pass
else:
    from importlib import reload

try:
    from . import test_modules
    reload(test_modules)
    from .test_modules import *
except ImportError:
    traceback.print_exc()
    print("Error importing test_modules, will not be available")
    input("Press enter to continue > ")

try:
    from . import PointClickNavModule
    reload(PointClickNavModule)
    from .PointClickNavModule import PointClickNav
except ImportError:
    traceback.print_exc()
    print("Error importing PointClickNavModule, will not be available")
    input("Press enter to continue > ")

try:
    from . import PointClickGraspModule
    reload(PointClickGraspModule)
    from .PointClickGraspModule import PointClickGrasp
except ImportError:
    traceback.print_exc()
    print("Error importing PointClickGraspModule, will not be available")
    input("Press enter to continue > ")

try:
    from . import DirectTeleOperationModule
    reload(DirectTeleOperationModule)
    from .DirectTeleOperationModule import DirectTeleOperation
except ImportError:
    traceback.print_exc()
    print("Error importing DirectTeleOperationModule, will not be available")
    input("Press enter to continue > ")
