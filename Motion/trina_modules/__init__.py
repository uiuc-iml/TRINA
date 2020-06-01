from importlib import reload
import trina_modules.test1
import trina_modules.test2

test1 = reload(test1)
test2 = reload(test2)

from trina_modules.test1 import *
from trina_modules.test2 import *

