from SimpleWebSocketServer import SimpleWebSocketServer, WebSocket
import pprint
import json
class SimpleEcho(WebSocket):

    def handleMessage(self):
        # echo message back to client
        obj = json.loads(self.data)
        pprint.pprint(obj,depth=4)

    def handleConnected(self):
        print(self.address, 'connected')

    def handleClose(self):
        print(self.address, 'closed')

server = SimpleWebSocketServer('130.126.139.236', 1234, SimpleEcho)
server.serveforever()