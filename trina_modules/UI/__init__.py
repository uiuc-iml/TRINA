import os,sys

if(sys.version_info[0] < 3):
    # from future import *
    from imp import reload

else:
    from importlib import reload
    
path = os.path.expanduser('~/TRINA/Motion/trina_modules/UI')
sys.path.append(path)

import RemoteUI
RemoteUI = reload(RemoteUI)
from RemoteUI import RemoteUIModule

import LocalUI
LocalUI = reload(LocalUI)
from LocalUI import LocalUIModule