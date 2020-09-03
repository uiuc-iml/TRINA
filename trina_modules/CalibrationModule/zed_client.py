import xmlrpc.client
class ZedClient:
    def __init__(self, address = 'http://localhost:8080'):
        self.s = xmlrpc.client.ServerProxy(address)

    def startCamera(self):
        self.s.startCamera()

    def closeCamera(self):
        self.s.closeCamera()

    def getCameraTransform(self):
        return self.s.getCameraTransform()

    def getPicture(self):
        return self.s.getPicture()


if __name__=="__main__":
    import time
    zed = ZedClient()
    zed.startCamera()
    print('calling get picture')
    start_time = time.time()
    pic = zed.getPicture()
    print('elapsed:',time.time() - start_time)
    zed.closeCamera()
