from trina import jarvis

class ExampleAPI(jarvis.APILayer):
    """Matches up with the items exposed in ExampleModule"""
    
    def loopCount(self):
        """Returns the number of times the loop has been called"""
        return self._redisGet(["Example","loopCount"])

    def add(self,a,b):
        return self._redisRpc("add",a,b)

    def sin(self,x):
        return self._redisRpc("sin",x)

    def sineWave(self):
        return self._redisRpc("sineWave")
    