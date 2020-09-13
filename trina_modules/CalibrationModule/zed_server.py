from xmlrpc.server import SimpleXMLRPCServer
from xmlrpc.server import SimpleXMLRPCRequestHandler
import sys
import signal
sys.path.append('../../')
from SensorModule import Camera_Robot
sys.path.append('../../Motion/')
import TRINAConfig


global camera
global server_started
server_started = False


def _startCamera():
    global camera
    global server_started

    if server_started:
        print("camera already started ")
    else:
        camera = Camera_Robot(robot = [],world = [], cameras =['zed_overhead'],ros_active = False, use_jarvis = False, mode = 'Physical')
    server_started = True
    print("camera started")
    return 0

def _closeCamera():
    global camera,server_started
    camera.safely_close_all()
    server_started = False
    return 0

def _getCameraTransform():
    return TRINAConfig.fixed_camera_transform

def _getPicture():
    global camera
    res = camera.get_rgbd_images()
    res['zed_overhead'][0]
    return str(res['zed_overhead'][0].tolist())


ip_address = '10.0.242.158'#'localhost'
port = 8040
server = SimpleXMLRPCServer((ip_address,port), logRequests=False)
server.register_introspection_functions()
# signal.signal(signal.SIGINT, sigint_handler)

server.register_function(_startCamera,'startCamera')
server.register_function(_closeCamera,'closeCamera')
server.register_function(_getCameraTransform,'getCameraTransform')
server.register_function(_getPicture,'getPicture')

print('Server Created')
server.serve_forever()
