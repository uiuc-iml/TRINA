import os
import signal
import sys
import time
import math
from threading import Thread, Lock
import threading
from limbController import LimbController
from baseController import BaseController
from gripperController import GripperController
from kinematicController import KinematicController
from torsoController import TorsoController
import TRINAConfig #network configs and other configs
from motionStates import * #state structures
#import convenienceFunctions
from copy import deepcopy
from klampt.math import vectorops,so3
from klampt import vis
from klampt.model import ik, collide
import numpy as np
from klampt import WorldModel
dirname = os.path.dirname(__file__)
#getting absolute model name
model_name = os.path.join(dirname, "data/TRINA_reflex.xml")
world = WorldModel()
res = world.readFile(model_name)