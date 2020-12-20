import sys
import time
import websocket
from threading import Thread
import time
import sys
import json

roomname = "The Lobby"
zonename = "BasicExamples"
userId=0
roomId=-1
is_closed=0




def on_message(ws, message):
    global userId,roomId,drone,roomname, zonename
    global is_closed
    print "Received ::::::: '%s'" % message
    mjson = json.loads(message)
    if mjson["a"]==0:
       a= {"a":1,"c":0,"p":{"zn": zonename,"un":"","pw":""}}
       b= json.dumps(a).encode('utf-8')
       ws.send(b)

    if mjson["a"]==1:
       if 'id' in mjson["p"]:
           print "userId: %d" % mjson["p"]["id"]
           userId = mjson["p"]["id"]
           a={"a":4,"c":0,"p":{"n":roomname}}
           b= json.dumps(a).encode('utf-8')
           ws.send(b)

    if mjson["a"]==1001: #Once after room jon sending welcome message
       if roomId ==-1:
           roomId = mjson["p"]["r"]
           print "Room id :::: '%s'" % roomId
        
           #sending public messages  
	   a={"a":7,"c":0,"p":{"t":0,"r":roomId,"u":userId,"m":"controllers","p":{"LeftRotation":{"x":0.1,"y":0.2,"z":0.3},"LeftPosition":{"x":1.1,"y":1.2,"z":1.3},"RightRotation":{"x":2.1,"y":2.2,"z":2.3},"RightPosition":{"x":3.1,"y":3.2,"z":3.3}}}}
           b= json.dumps(a).encode('utf-8')
           ws.send(b)

                                           
    if mjson["a"]==7:  #Retrieve public message
       if mjson["p"]["m"]=="controllers" and mjson["p"]["r"]==roomId:
           print("LeftRotation x:(%s) y:(%s) z:(%s)" %(mjson["p"]["p"]["LeftRotation"]["x"],mjson["p"]["p"]["LeftRotation"]["y"],mjson["p"]["p"]["LeftRotation"]["z"]))
           print("LeftPosition x:(%s) y:(%s) z:(%s)" %(mjson["p"]["p"]["LeftPosition"]["x"],mjson["p"]["p"]["LeftPosition"]["y"],mjson["p"]["p"]["LeftPosition"]["z"]))
           print("RightRotation x:(%s) y:(%s) z:(%s)" %(mjson["p"]["p"]["RightRotation"]["x"],mjson["p"]["p"]["RightRotation"]["y"],mjson["p"]["p"]["RightRotation"]["z"]))
           print("RightPosition x:(%s) y:(%s) z:(%s)" %(mjson["p"]["p"]["RightPosition"]["x"],mjson["p"]["p"]["RightPosition"]["y"],mjson["p"]["p"]["RightPosition"]["z"]))
          
     


def on_error(ws, error):
    print(error)


def on_close(ws):
    global is_closed
    is_closed=1
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

    Thread(target=run).start()




if __name__ == "__main__":
    websocket.enableTrace(True)
    host = "ws://gametest.vrotors.com:8888/websocket"
    ws = websocket.WebSocketApp(host,
                                on_message=on_message,
                                on_error=on_error,
                                on_close=on_close)
    ws.on_open = on_open
    ws.run_forever()

###


