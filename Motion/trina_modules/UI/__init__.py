from importlib import reload
import os,sys

path = os.path.expanduser('~/TRINA/Motion/trina_modules/UI')
sys.path.append(path)

import UI

UI = reload(UI)

from UI import UI