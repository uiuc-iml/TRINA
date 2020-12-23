from trina import jarvis
from klampt import io

class UIAPI(jarvis.APILayer):
    @classmethod
    def name(self):
        return "ui"

    def getRayClick(self):

        """once this function is called, the UI will ask the user to click twice on the map, and sending back 
        2 ray objects according to the user clicks. first one for destination, second one for calibration
        return:
            jarvis.RpcPromise, which will give None if the user has canceled, but otherwise will give a pair
            of rays in the format
            {
                'FIRST_RAY': {'destination': [-0.8490072256426063,-0.2846905378876157,-0.4451269801347757],
                            'source': [12.653596500469428, 1.6440497080649081, 5.851982763380186]},
                'SECOND_RAY': {'destination': [-0.8590257360168888,-0.20712234383654582,-0.46816142466493127],
                            'source': [12.653596500469428, 1.6440497080649081, 5.851982763380186]}
            }
        blocking?:
            no
        """
        return self._redisRpc('getRayClick')

    def addTextUI(self, name, text, color, size):
        """add text to specfified location on UI screen. 
        args:
            name: (str) id for the text object
            text: (str) content you wish to add
            color: (list) rgb value [0,0,0]
            size: (int) font size
        return:
            None
        blocking?:
            no
        """
        return self._redisRpcNoReply('addText',name,text,color,size)

    def sendConfirmationUI(self,title,text):
        """once this function is called, the UI will display a confimation window with specified title and text, 
        
        return:
            jarvis.RpcPromise, which will give a string, either 'YES' or 'NO'
        blocking?:
            no
        """
        return self._redisRpc('addConfirmation', title, text)
        
    def sendTrajectoryUI(self,trajectory,animate = False):
        """send a trajectory to UI, UI will add the path preview and animate? the robot ghost immediately for only once 
        
        args:
            trajectory: (klampt obj) the traj calculated
            animate: (bool) if user wants to animate the path
        return:
            None
        blocking?:
            no
        """
        trajectory = io.loader.toJson(trajectory, 'Trajectory')
        return self._redisRpcNoReply('sendTrajectory',trajectory,animate)

    def addButtonUI(self, name, text):
        """add a button to the UI window
        args:
            name: (str)  id for the button object
            text: (str) button label text
        return:
            None        
        blocking?:
            no
        """
        self._redisRpcNoReply('addButton',name,text)

    def getButtonClickUI(self,name):
        """returns True if button with specified name is clicked
        args:
            name: (str) id for the button object
        return:
            (bool) True or False
        
        blocking?:
            no
        """
        return self._redisGet(['UI_STATE',name])

    def addPromptUI(self,title,text):
        id = '$'+ uuid.uuid1().hex
        # TODO
        return  id

    def addInputBoxUI(self,title,text,fields):
        id = '$'+ uuid.uuid1().hex
        # TODO
        return id

    def getSaveConfigSignalUI(self):
        if not self.server['UI_FEEDBACK']['move-to-save-signal']['REPLIED'].read():
            return 'NOT READY'
        else:
            name =  self.server['UI_FEEDBACK']['move-to-save-signal']['MSG'].read()
            self.server['UI_FEEDBACK']['move-to-save-signal']['REPLIED'] = False
            return True, name

    def getLoadConfigSignalUI(self):
        if not self.server['UI_FEEDBACK']['move-to-load-signal']['REPLIED'].read():
            return 'NOT READY'
        else:
            name =  self.server['UI_FEEDBACK']['move-to-load-signal']['MSG'].read()
            self.server['UI_FEEDBACK']['move-to-load-signal']['REPLIED'] = False
            return True, name

    def addRobotTelemetry(self, value):
        """ Add UI robot telemetry value
        """
        self._redisSet(["robotTelemetry"],value)
