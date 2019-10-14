using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using WebSocketSharp;

public class OutputState : MonoBehaviour
{
    [System.Serializable]
    public class LeftController
    {
        public List<bool> press;
        public List<bool> touch;
        public List<bool> nearTouch;
        public List<float> squeeze;
        public List<float> thumbstickMovement;
        public static LeftController ToLeftController()
        {
            return new LeftController
            {
                press = new List<bool> {false, false, false, false , false, false},
                touch = new List<bool> { false, false, false, false, false, false, false, false},
                nearTouch = new List<bool> { false, false, false },
                squeeze = new List<float> {0,0},
                thumbstickMovement = new List<float> {0,0}
            };
        }
    }
    [System.Serializable]
    public class RightController
    {
        public List<bool> press;
        public List<bool> touch;
        public List<bool> nearTouch;
        public List<float> squeeze;
        public List<float> thumbstickMovement;
        public static RightController ToRightController()
        {
            return new RightController
            {
                press = new List<bool> { false, false, false, false, false, false },
                touch = new List<bool> { false, false, false, false, false, false, false, false },
                nearTouch = new List<bool> { false, false, false },
                squeeze = new List<float> { 0, 0 },
                thumbstickMovement = new List<float> { 0, 0 }
            };
        }
    }
    [System.Serializable]
    public class ControllerButtonState
    {
        public LeftController leftController;
        public RightController rightController;
        public static ControllerButtonState ToControllerButtonState()
        {
            return new ControllerButtonState
            {
                leftController = LeftController.ToLeftController(),
                rightController = RightController.ToRightController()
            };
        }
    }
    [System.Serializable]
    public class HeadSetPositionState
    {
        public List<float> deviceRotation;
        public List<float> devicePosition;
        public List<float> neckPositoin;
        public static HeadSetPositionState ToHeadSetPositionState()
        {
            return new HeadSetPositionState
            {
                devicePosition = new List<float> {0,0,0},
                deviceRotation = new List<float> {0,0},
                neckPositoin = new List<float> {0,0,0}
            };
        }
    }
    [System.Serializable]
    public class RightController2
    {
        public List<float> controllerOrientation;
        public List<float> controllerPosition;
        public static RightController2 ToRightController2()
        {
            return new RightController2
            {
                controllerOrientation = new List<float> { 0, 0 },
                controllerPosition = new List<float> { 0, 0, 0 }
            };
        }
    }
    [System.Serializable]
    public class LeftController2
    {
        public List<float> controllerOrientation;
        public List<float> controllerPosition;
        public static LeftController2 ToLeftController2()
        {
            return new LeftController2
            {
                controllerOrientation = new List<float> { 0, 0 },
                controllerPosition = new List<float> { 0, 0, 0 }
            };
        }
    }
    [System.Serializable]
    public class ControllerPositionState
    {
        public LeftController2 leftController;
        public RightController2 rightController;
        public static ControllerPositionState ToControllerPositionState()
        {
            return new ControllerPositionState
            {
                leftController = LeftController2.ToLeftController2(),
                rightController = RightController2.ToRightController2()
            };
        }
    }
    [System.Serializable]
    public class UIlogicState
    {
        public bool stop;
        public bool teleoperationMode;
        public bool autonomousMode;
        public static UIlogicState ToUIlogicState()
        {
            return new UIlogicState
            {
                stop = false,
                teleoperationMode = false,
                autonomousMode = false
            };
        }
    }
    [System.Serializable]
    public class UIOutput
    {
        public string title= "UI Outputs";
        public ControllerButtonState controllerButtonState;
        public HeadSetPositionState headSetPositionState;
        public ControllerPositionState controllerPositionState;
        public UIlogicState UIlogicState;
        public static UIOutput ToUIOutput()
        {
            return new UIOutput
            {
                controllerButtonState = ControllerButtonState.ToControllerButtonState(),
                headSetPositionState = HeadSetPositionState.ToHeadSetPositionState(),
                controllerPositionState = ControllerPositionState.ToControllerPositionState(),
                UIlogicState = UIlogicState.ToUIlogicState()
            };
        }
    }

    private WebSocket ws;
    private int frameCount = 0;
    public UIOutput outputState = UIOutput.ToUIOutput();
 
    void Start()
    {

        Debug.Log("initiate WebSocket");
        //initiate websocket Connection
        //ws = new WebSocket("ws://130.126.138.139:9000");
        ws = new WebSocket("ws://130.126.138.139:1234");

        ws.OnMessage += (sender, e) =>
        {
            Debug.Log($"OnMessage----->{e.Data}");
        };

        ws.OnOpen += (sender, e) =>
        {
            Debug.Log($"OnOpen");
        };

        ws.OnClose += (sender, e) =>
        {
            Debug.Log("onClose------>WebSocket closed with reason: " + e.Reason);
        };

        ws.Connect();
    }

    void Update()
    {
        frameCount++;
        if (frameCount == 300) {
            ws.Connect();
            frameCount = 0;
            ws.Send(JsonUtility.ToJson(outputState));
        }
    }

    public void DummyUpdateState()
    {
        outputState.UIlogicState.stop = !outputState.UIlogicState.stop;
        outputState.controllerButtonState.leftController.press[0] = true;
    }

    private void SendWS(string data)
    {
        ws.Send(data);
    }

}
