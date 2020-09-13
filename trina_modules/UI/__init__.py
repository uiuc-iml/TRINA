import os,sys

if(sys.version_info[0] < 3):
    # from future import *
    from imp import reload

else:
    from importlib import reload
    
path = os.path.expanduser('~/TRINA/Motion/trina_modules/UI')
sys.path.append(path)

import UI

UI = reload(UI)

from UI import UI