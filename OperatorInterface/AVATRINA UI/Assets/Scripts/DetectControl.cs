using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;


public class DetectControl : MonoBehaviour
{
    private Text text;
    private List<string> detectionHistory;
    private int lineNum;
    // Start is called before the first frame update
    void Start()
    {
        text = this.GetComponent<Text>();
        detectionHistory = new List<string>();
        lineNum = 0;
    }

    // Update is called once per frame
    void Update()
    {
        if (OVRInput.GetControllerWasRecentered())
        {
            detectionHistory.Add("Controller was Recentered\n");
            lineNum++;
        }

        //Button/Touch.One
        else if (OVRInput.Get(OVRInput.Button.One))
        {
            detectionHistory.Add("Get(Button.One)\n");
            lineNum++;
        }
        else if (OVRInput.GetDown(OVRInput.Button.One))
        {
            detectionHistory.Add("GetDown(Button.One)\n");
            lineNum++;
        }
        else if (OVRInput.GetUp(OVRInput.Button.One))
        {
            detectionHistory.Add("GetUp(Button.One)\n");
            lineNum++;
        }
        else if (OVRInput.Get(OVRInput.Touch.One))
        {
            detectionHistory.Add("Get(Touch.One)\n");
            lineNum++;
        }
        else if (OVRInput.GetDown(OVRInput.Touch.One))
        {
            detectionHistory.Add("GetDown(Touch.One)\n");
            lineNum++;
        }
        else if (OVRInput.GetUp(OVRInput.Touch.One))
        {
            detectionHistory.Add("GetUp(Touch.One)\n");
            lineNum++;
        }

        //Button/Touch.Two
        else if (OVRInput.Get(OVRInput.Button.Two))
        {
            detectionHistory.Add("Get(Button.Two)\n");
            lineNum++;
        }
        else if (OVRInput.GetDown(OVRInput.Button.Two))
        {
            detectionHistory.Add("GetDown(Button.Two)\n");
            lineNum++;
        }
        else if (OVRInput.GetUp(OVRInput.Button.Two))
        {
            detectionHistory.Add("GetUp(Button.Two)\n");
            lineNum++;
        }
        else if (OVRInput.Get(OVRInput.Touch.Two))
        {
            detectionHistory.Add("Get(Touch.Two)\n");
            lineNum++;
        }
        else if (OVRInput.GetDown(OVRInput.Touch.Two))
        {
            detectionHistory.Add("GetDown(Touch.Two)\n");
            lineNum++;
        }
        else if (OVRInput.GetUp(OVRInput.Touch.Two))
        {
            detectionHistory.Add("GetUp(Touch.Two)\n");
            lineNum++;
        }

        //Button/Touch.Three
        else if (OVRInput.Get(OVRInput.Button.Three))
        {
            detectionHistory.Add("Get(Button.Three)\n");
            lineNum++;
        }
        else if (OVRInput.GetDown(OVRInput.Button.Three))
        {
            detectionHistory.Add("GetDown(Button.Three)\n");
            lineNum++;
        }
        else if (OVRInput.GetUp(OVRInput.Button.Three))
        {
            detectionHistory.Add("GetUp(Button.Three)\n");
            lineNum++;
        }
        else if (OVRInput.Get(OVRInput.Touch.Three))
        {
            detectionHistory.Add("Get(Touch.Three)\n");
            lineNum++;
        }
        else if (OVRInput.GetDown(OVRInput.Touch.Three))
        {
            detectionHistory.Add("GetDown(Touch.Three)\n");
            lineNum++;
        }
        else if (OVRInput.GetUp(OVRInput.Touch.Three))
        {
            detectionHistory.Add("GetUp(Touch.Three)\n");
            lineNum++;
        }

        //Button/Touch.Four
        else if (OVRInput.Get(OVRInput.Button.Four))
        {
            detectionHistory.Add("Get(Button.Four)\n");
            lineNum++;
        }
        else if (OVRInput.GetDown(OVRInput.Button.Four))
        {
            detectionHistory.Add("GetDown(Button.Four)\n");
            lineNum++;
        }
        else if (OVRInput.GetUp(OVRInput.Button.Four))
        {
            detectionHistory.Add("GetUp(Button.Four)\n");
            lineNum++;
        }
        else if (OVRInput.Get(OVRInput.Touch.Four))
        {
            detectionHistory.Add("Get(Touch.Four)\n");
            lineNum++;
        }
        else if (OVRInput.GetDown(OVRInput.Touch.Four))
        {
            detectionHistory.Add("GetDown(Touch.Four)\n");
            lineNum++;
        }
        else if (OVRInput.GetUp(OVRInput.Touch.Four))
        {
            detectionHistory.Add("GetUp(Touch.Four)\n");
            lineNum++;
        }

        //Button/Touch.PrimaryIndexTrigger
        else if (OVRInput.Get(OVRInput.Button.PrimaryIndexTrigger))
        {
            detectionHistory.Add("Get(Button.PrimaryIndexTrigger)\n");
            lineNum++;
        }
        else if (OVRInput.GetDown(OVRInput.Button.PrimaryIndexTrigger))
        {
            detectionHistory.Add("GetDown(Button.PrimaryIndexTrigger)\n");
            lineNum++;
        }
        else if (OVRInput.GetUp(OVRInput.Button.PrimaryIndexTrigger))
        {
            detectionHistory.Add("GetUp(Button.PrimaryIndexTrigger)\n");
            lineNum++;
        }
        else if (OVRInput.Get(OVRInput.Touch.PrimaryIndexTrigger))
        {
            detectionHistory.Add("Get(Touch.PrimaryIndexTrigger)\n");
            lineNum++;
        }
        else if (OVRInput.GetDown(OVRInput.Touch.PrimaryIndexTrigger))
        {
            detectionHistory.Add("GetDown(Touch.PrimaryIndexTrigger)\n");
            lineNum++;
        }
        else if (OVRInput.GetUp(OVRInput.Touch.PrimaryIndexTrigger))
        {
            detectionHistory.Add("GetUp(Touch.PrmiaryInexTrigger)\n");
            lineNum++;
        }

        //Button/Touch.SecondaryIndexTrigger
        else if (OVRInput.Get(OVRInput.Button.SecondaryIndexTrigger))
        {
            detectionHistory.Add("Get(Button.SecondaryIndexTrigger)\n");
            lineNum++;
        }
        else if (OVRInput.GetDown(OVRInput.Button.SecondaryIndexTrigger))
        {
            detectionHistory.Add("GetDown(Button.SecondaryIndexTrigger)\n");
            lineNum++;
        }
        else if (OVRInput.GetUp(OVRInput.Button.SecondaryIndexTrigger))
        {
            detectionHistory.Add("GetUp(Button.SecondaryIndexTrigger)\n");
            lineNum++;
        }
        else if (OVRInput.Get(OVRInput.Touch.SecondaryIndexTrigger))
        {
            detectionHistory.Add("Get(Touch.SecondaryIndexTrigger)\n");
            lineNum++;
        }
        else if (OVRInput.GetDown(OVRInput.Touch.SecondaryIndexTrigger))
        {
            detectionHistory.Add("GetDown(Touch.SecondaryIndexTrigger)\n");
            lineNum++;
        }
        else if (OVRInput.GetUp(OVRInput.Touch.SecondaryIndexTrigger))
        {
            detectionHistory.Add("GetUp(Touch.SecondaryIndexTrigger)\n");
            lineNum++;
        }

        //Button.PrimaryHandTrigger
        else if (OVRInput.Get(OVRInput.Button.PrimaryHandTrigger))
        {
            detectionHistory.Add("Get(Button.PrimaryHandTrigger)\n");
            lineNum++;
        }
        else if (OVRInput.GetDown(OVRInput.Button.PrimaryHandTrigger))
        {
            detectionHistory.Add("GetDown(Button.PrimaryHandTrigger)\n");
            lineNum++;
        }
        else if (OVRInput.GetUp(OVRInput.Button.PrimaryHandTrigger))
        {
            detectionHistory.Add("GetUp(Button.PrimaryHandTrigger)\n");
            lineNum++;
        }

        //Button.SecondaryHandTrigger
        else if (OVRInput.Get(OVRInput.Button.SecondaryHandTrigger))
        {
            detectionHistory.Add("Get(Button.SecondaryHandTrigger)\n");
            lineNum++;
        }
        else if (OVRInput.GetDown(OVRInput.Button.SecondaryHandTrigger))
        {
            detectionHistory.Add("GetDown(Button.SecondaryHandTrigger)\n");
            lineNum++;
        }
        else if (OVRInput.GetUp(OVRInput.Button.SecondaryHandTrigger))
        {
            detectionHistory.Add("GetUp(Button.SecondaryHandTrigger)\n");
            lineNum++;
        }

        //Button.PrimaryThumbstick
        else if (OVRInput.Get(OVRInput.Button.PrimaryThumbstick))
        {
            detectionHistory.Add("Get(Button.PrimaryThumbstick)\n");
            lineNum++;
        }
        else if (OVRInput.GetDown(OVRInput.Button.PrimaryThumbstick))
        {
            detectionHistory.Add("GetDown(Button.PrimaryThumbstick)\n");
            lineNum++;
        }
        else if (OVRInput.GetUp(OVRInput.Button.PrimaryThumbstick))
        {
            detectionHistory.Add("GetUp(Button.PrimaryThumbstick)\n");
            lineNum++;
        }
        else if (OVRInput.Get(OVRInput.Button.PrimaryThumbstickUp))
        {
            detectionHistory.Add("Get(Button.PrimaryThumbstickUp)\n");
            lineNum++;
        }
        else if (OVRInput.Get(OVRInput.Button.PrimaryThumbstickDown))
        {
            detectionHistory.Add("Get(Button.PrimaryThumbstickDown)\n");
            lineNum++;
        }
        else if (OVRInput.Get(OVRInput.Button.PrimaryThumbstickLeft))
        {
            detectionHistory.Add("Get(Button.PrimaryThumbstickLeft)\n");
            lineNum++;
        }
        else if (OVRInput.Get(OVRInput.Button.PrimaryThumbstickRight))
        {
            detectionHistory.Add("Get(Button.PrimaryThumbstickRight)\n");
            lineNum++;
        }


        //Button.SecondaryThumbstick
        else if (OVRInput.Get(OVRInput.Button.SecondaryThumbstick))
        {
            detectionHistory.Add("Get(Button.SecondaryThumbstick)\n");
            lineNum++;
        }
        else if (OVRInput.GetDown(OVRInput.Button.SecondaryThumbstick))
        {
            detectionHistory.Add("GetDown(Button.SecondaryThumbstick)\n");
            lineNum++;
        }
        else if (OVRInput.GetUp(OVRInput.Button.SecondaryThumbstick))
        {
            detectionHistory.Add("GetUp(Button.SecondaryThumbstick)\n");
            lineNum++;
        }
        else if (OVRInput.Get(OVRInput.Button.SecondaryThumbstickUp))
        {
            detectionHistory.Add("Get(Button.SecondaryThumbstickUp)\n");
            lineNum++;
        }
        else if (OVRInput.Get(OVRInput.Button.SecondaryThumbstickDown))
        {
            detectionHistory.Add("Get(Button.SecondaryThumbstickDown)\n");
            lineNum++;
        }
        else if (OVRInput.Get(OVRInput.Button.SecondaryThumbstickLeft))
        {
            detectionHistory.Add("Get(Button.SecondaryThumbstickLeft)\n");
            lineNum++;
        }
        else if (OVRInput.Get(OVRInput.Button.SecondaryThumbstickRight))
        {
            detectionHistory.Add("Get(Button.SecondaryThumbstickRight)\n");
            lineNum++;
        }

        //Button.Up/Down/Left/Right
        else if (OVRInput.Get(OVRInput.Button.Up))
        {
            detectionHistory.Add("Get(Button.Up)\n");
            lineNum++;
        }
        else if (OVRInput.Get(OVRInput.Button.Down))
        {
            detectionHistory.Add("Get(Button.Down)\n");
            lineNum++;
        }
        else if (OVRInput.Get(OVRInput.Button.Left))
        {
            detectionHistory.Add("Get(Button.Left)\n");
            lineNum++;
        }
        else if (OVRInput.Get(OVRInput.Button.Right))
        {
            detectionHistory.Add("Get(Button.Right)\n");
            lineNum++;
        }

        //Button/Touch.PrimaryTouchpad
        else if (OVRInput.Get(OVRInput.Button.PrimaryTouchpad))
        {
            detectionHistory.Add("Get(Button.PrimaryTouchpad)\n");
            lineNum++;
        }
        else if (OVRInput.GetDown(OVRInput.Button.PrimaryTouchpad))
        {
            detectionHistory.Add("GetDown(Button.PrimaryTouchpad)\n");
            lineNum++;
        }
        else if (OVRInput.GetUp(OVRInput.Button.PrimaryTouchpad))
        {
            detectionHistory.Add("GetUp(Button.PrimaryTouchpad)\n");
            lineNum++;
        }
        else if (OVRInput.Get(OVRInput.Touch.PrimaryTouchpad))
        {
            detectionHistory.Add("Get(Touch.PrimaryTouchpad)\n");
            lineNum++;
        }
        else if (OVRInput.GetDown(OVRInput.Touch.PrimaryTouchpad))
        {
            detectionHistory.Add("GetDown(Touch.PrimaryTouchpad)\n");
            lineNum++;
        }
        else if (OVRInput.GetUp(OVRInput.Touch.PrimaryTouchpad))
        {
            detectionHistory.Add("GetUp(Touch.PrimaryTouchpad)\n");
            lineNum++;
        }

        //RawButtons.Start
        else if (OVRInput.Get(OVRInput.RawButton.Start))
        {
            detectionHistory.Add("Get(RawButton.Start)\n");
            lineNum++;
        }
        else if (OVRInput.GetDown(OVRInput.RawButton.Start))
        {
            detectionHistory.Add("GetDown(RawButton.Start)\n");
            lineNum++;
        }
        else if (OVRInput.GetUp(OVRInput.RawButton.Start))
        {
            detectionHistory.Add("GetUp(RawButton.Start)\n");
            lineNum++;
        }

        HistoryToText();
    }

    void HistoryToText()
    {
        if (lineNum > 20 || detectionHistory.Count > 20)
        {
            lineNum = 0;
            detectionHistory.Clear();
        }
        text.text = "";
        foreach (var line in detectionHistory)
        {
            text.text += line;
        }
    }
}
