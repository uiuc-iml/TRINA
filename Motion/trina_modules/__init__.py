from importlib import reload
import os,sys

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
