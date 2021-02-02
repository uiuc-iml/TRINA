import os,sys

from .motion_client import MotionClient

<<<<<<< HEAD
if(sys.version_info[0] < 3):
    # from future import *
    from .motion_client import MotionClient
else:
    from .motion_client_python3 import MotionClient
=======
>>>>>>> 3fd839e90dc13183966930e65a1e899e1556d45a
from . import TRINAConfig

