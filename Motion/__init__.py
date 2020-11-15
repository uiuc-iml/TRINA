import os,sys

import os,sys

if(sys.version_info[0] < 3):
    # from future import *
    from .motion_client import MotionClient
else:
    from .motion_client_python3 import MotionClient
from . import TRINAConfig

