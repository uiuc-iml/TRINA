import os,sys
import traceback

if(sys.version_info[0] < 3):
    # from future import *
    pass
else:
    from importlib import reload

try:
    from . import LoggerModule
    reload(LoggerModule)
    from .LoggerModule import StateLogger
except ImportError:
    traceback.print_exc()
    print("Error importing LoggerModule, will not be available")
    input("Press enter to continue > ")

# UI = reload(trina_modules.UI.UI)
# from trina_modules.UI.UI import *
