import os, sys
if(sys.version_info[0] < 3):
    # from future import *
    from imp import reload

else:
    from importlib import reload

path = os.path.expanduser('~/TRINA/Motion/')
sys.path.append(path)




from . import CalibrationFile
reload(PointClickNavFile)

#TODO
from CalibrationFile import 

sys.path.remove(path)