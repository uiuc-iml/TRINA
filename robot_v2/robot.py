import sys
import time
import websocket
from threading import Thread
import time
import sys
import json
from reem.connection import RedisInterface
from reem.datatypes import KeyValueStore
from unidecode import unidecode
roomname = "The Lobby"
zonename = "BasicExamples"
userId = 0
roomId = -1
is_closed = 0


def on_message(ws, message):
    global userId, roomId, drone, roomname, zonename
    global is_closed
    global server
    global UI_STATE
    global counter
    global paused, puase_button_pressed
    # Python2 compatibility
    if type(message) != str:
        message = unidecode(message)
    mjson = json_loads_byteified(message)
    # print(mjson)
    if mjson["a"] == 0:
        a = {"a": 1, "c": 0, "p": {"zn": zonename, "un": "", "pw": ""}}
        b = json.dumps(a).encode('utf-8')
        ws.send(b)

    if mjson["a"] == 1:
        if 'id' in mjson["p"]:
            # print("userId: %d" % mjson["p"]["id"])
            userId = mjson["p"]["id"]
            a = {"a": 4, "c": 0, "p": {"n": roomname}}
            b = json.dumps(a).encode('utf-8')
            ws.send(b)

    if mjson["a"] == 1001:  # Once after room jon sending welcome message
        if roomId == -1:
            roomId = mjson["p"]["r"]
            # print("Room id :::: '%s'" % roomId)

            # robotrec = {"title": "Robot Telemetry Data", "status": {"healthy": "True", "eStop": "False", "softwareEStop": "False"}, "currentConfig": {"leftArm": [-0.20578666666666667, -2.1202733333333335, -1.6030666666666669, 3.7186333333333335, -0.96508, 0.0974], "rightArm": [0.2028, -1.035932653589793, 1.6057333333333335, -0.5738406797435401, 0.9622, -0.0974], "torso": [0.1, 1.1], "baseOdometry": [0.1, 0.2, 0.4]}, "currentVelocity": {"leftArm": [0, 0, 0, 0, 0, 0], "rightArm": [
            #     0, 0, 0, 0, 0, 0], "baseWheels": [1.1, 0.2], "base": [-0.5, 0.5, 0]}, "targetConfig": {"leftArm": [0.1, 1.1, 2.1, 3.1, 4.1, 5.1], "rightArm": [0.1, 1.1, 2.1, 3.1, 4.1, 5.1], "torso": [0.1, 1.1], "baseOdometry": [0.1, 0.2, 0.4]}, "controller": {"collisionWarning": {"leftArm": "True", "rightArm": "False"}, "unreachableWarning": {"leftArm": "True", "rightArm": "False"}}, "perception": {"miniMap": {"width": 2, "height": 1, "matrix": [[0], [0]]}, "hapticFeedback": [0, 1, 2]}}

            # # sending public messages
            # a = {"a": 7, "c": 0, "p": {"t": 0, "r": roomId,
            #                            "u": userId, "m": "robot_telemetry", "p": robotrec}}

            # b = json.dumps(a).encode('utf-8')
            # ws.send(b)

    if mjson["a"] == 7:  # Retrieve public message
        if mjson["p"]["m"] == "controllers" and mjson["p"]["r"] == roomId:
            robotrec = {"title": "Robot Telemetry Data", "status": {"healthy": "True", "eStop": "False", "softwareEStop": "False"}, "currentConfig": {"leftArm": [-0.20578666666666667, -2.1202733333333335, -1.6030666666666669, 3.7186333333333335, -0.96508, 0.0974], "rightArm": [0.2028, -1.035932653589793, 1.6057333333333335, -0.5738406797435401, 0.9622, -0.0974], "torso": [0.1, 1.1], "baseOdometry": [0.1, 0.2, 0.4]}, "currentVelocity": {"leftArm": [0, 0, 0, 0, 0, 0], "rightArm": [
                0, 0, 0, 0, 0, 0], "baseWheels": [1.1, 0.2], "base": [-0.5, 0.5, 0]}, "targetConfig": {"leftArm": [0.1, 1.1, 2.1, 3.1, 4.1, 5.1], "rightArm": [0.1, 1.1, 2.1, 3.1, 4.1, 5.1], "torso": [0.1, 1.1], "baseOdometry": [0.1, 0.2, 0.4]}, "controller": {"collisionWarning": {"leftArm": "True", "rightArm": "False"}, "unreachableWarning": {"leftArm": "True", "rightArm": "False"}}, "perception": {"miniMap": {"width": 2, "height": 1, "matrix": [[0], [0]]}, "hapticFeedback": [0, 1, 2]}}

            #handle robot pause logic
            server["UI_STATE"] = mjson["p"]["p"]
            if mjson["p"]["p"]["controllerButtonState"]["rightController"]["press"][1] and not puase_button_pressed:
                paused = not paused
                server["VR_Stop"] = paused
            puase_button_pressed = mjson["p"]["p"]["controllerButtonState"]["rightController"]["press"][1]

            return

            try:
                robot_telemetry = server['robotTelemetry'].read()

                if(type(robot_telemetry) != int):
                    a = 0
                    # robotrec["currentConfig"]["rightArm"] = robot_telemetry['rightArm']
                    # robotrec["currentConfig"]["leftArm"] = robot_telemetry['leftArm']
                    robotrec["targetConfig"]["rightArm"] = robot_telemetry['rightArm']
                    robotrec["targetConfig"]["leftArm"] = robot_telemetry['leftArm']
                # robotrec["currentConfig"]["rightArm"] = [0.0+counter*0.03,0.0,0.0,0.0,0.0,0.0]
                # robotrec["currentConfig"]["leftArm"] = [0.0+counter*0.03,0.0,0.0,0.0,0.0,0.0]
                # robotrec["targetConfig"]["rightArm"] = [0.0+counter*0.03,0.0,0.0,0.0,0.0,0.0]
                # robotrec["targetConfig"]["leftArm"] = [0.0+counter*0.03,0.0,0.0,0.0,0.0,0.0]
                    a = {"a": 7, "c": 0, "p": {"t": 0, "r": roomId,
                                                "u": userId, "m": "robot_telemetry", "p": robotrec}}
                    b = json.dumps(a).encode('utf-8')
                    # ws.send(b)
            except Exception as e:
                print('could not read robotTelemetry',e)


def on_error(ws, error):
    print(error)


def on_close(ws):
    global is_closed
    is_closed = 1
    print("### closed ###")


def on_open(ws):
    def run(*args):
        global is_closed
        a = {"a":0,"c":0,"p":{"api":"1.2.0","cl":"JavaScript"}}
        b =json.dumps(a).encode('utf-8')
        print(b)
        ws.send(b)

        while is_closed==0:
              time.sleep(1)
        time.sleep(1)
        ws.close()
        print("Thread terminating...")
        
        
    def ping(*args):
        global is_closed,roomId
        while is_closed==0:
             time.sleep(5)
             if roomId!=-1:
                  a={"a":7,"c":0,"p":{"t":0,"r":roomId,"u":userId,"m":"PONG","p":{}}}
                  b= json.dumps(a).encode('utf-8')
                  ws.send(b)
                  print("PONG...")
                     
        time.sleep(1)
        #ws.close()
        print("Ping Thread terminating...")

    Thread(target=run).start()
    Thread(target=ping).start()

"""Python2 compatibility: json.loads will store keys and string values as
unicode strings instead of str (byte) strings. This causes an issue in
REEM which checks that all keys are of type `str`. These functions allow for
loading json objects with byte strings.

Taken from here:
https://stackoverflow.com/a/33571117
"""
def json_load_byteified(file_handle):
    return _byteify(
        json.load(file_handle, object_hook=_byteify),
        ignore_dicts=True
    )


def json_loads_byteified(json_text):
    return _byteify(
        json.loads(json_text, object_hook=_byteify),
        ignore_dicts=True
    )


def _byteify(data, ignore_dicts = False):
    # if this is a unicode string, return its string representation
    if sys.version_info[0] < 3 and isinstance(data, unicode):
        return data.encode('utf-8')
    # if this is a list of values, return list of byteified values
    if isinstance(data, list):
        return [ _byteify(item, ignore_dicts=True) for item in data ]
    # if this is a dictionary, return dictionary of byteified keys and values
    # but only if we haven't already byteified it
    if isinstance(data, dict) and not ignore_dicts:
        return {
            _byteify(key, ignore_dicts=True): _byteify(value, ignore_dicts=True)
            for key, value in data.items()
        }
    # if it's anything else, return it in its original form
    return data

def listen():
    global userId, roomId, drone, roomname, zonename
    global is_closed
    global server
    global UI_STATE
    global counter
    global paused, puase_button_pressed
    counter = 0
    interface = RedisInterface(host="localhost")
    interface.initialize()
    server = KeyValueStore(interface)
    server['robotTelemetry'] = 0
    robotrec = {"title": "Robot Telemetry Data", "status": {"healthy": "True", "eStop": "False", "softwareEStop": "False"}, "currentConfig": {"leftArm": [-0.20578666666666667, -2.1202733333333335, -1.6030666666666669, 3.7186333333333335, -0.96508, 0.0974], "rightArm": [0.2028, -1.035932653589793, 1.6057333333333335, -0.5738406797435401, 0.9622, -0.0974], "torso": [0.1, 1.1], "baseOdometry": [0.1, 0.2, 0.4]}, "currentVelocity": {"leftArm": [0, 0, 0, 0, 0, 0], "rightArm": [
    0, 0, 0, 0, 0, 0], "baseWheels": [1.1, 0.2], "base": [-0.5, 0.5, 0]}, "targetConfig": {"leftArm": [0.1, 1.1, 2.1, 3.1, 4.1, 5.1], "rightArm": [0.1, 1.1, 2.1, 3.1, 4.1, 5.1], "torso": [0.1, 1.1], "baseOdometry": [0.1, 0.2, 0.4]}, "controller": {"collisionWarning": {"leftArm": "True", "rightArm": "False"}, "unreachableWarning": {"leftArm": "True", "rightArm": "False"}}, "perception": {"miniMap": {"width": 2, "height": 1, "matrix": [[0], [0]]}, "hapticFeedback": [0, 1, 2]}}

    UI_STATE = 0
    paused, puase_button_pressed = False, False
    websocket.enableTrace(True)
    host = "ws://gametest.vrotors.com:8888/websocket"
    ws = websocket.WebSocketApp(host,
                                on_message=on_message,
                                on_error=on_error,
                                on_close=on_close)
    ws.on_open = on_open
    ws.run_forever()

if __name__ == "__main__":
    listen()
###
