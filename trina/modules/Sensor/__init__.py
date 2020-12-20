import sys

if sys.version_info[0] < 3:
    pass    
else:
    from importlib import reload

import module
import api

reload(module)
reload(api)

from module import SensorModule
from api import JarvisSensorAPI

