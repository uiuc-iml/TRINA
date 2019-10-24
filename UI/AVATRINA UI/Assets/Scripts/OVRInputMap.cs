using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class OVRInputMap : MonoBehaviour
{
    private OutputState OutputState;
    public OVRGrabber left;
    public OVRGrabber right;
    public OVRCameraRig headset;
    void Start()
    {
        OutputState = gameObject.GetComponent<OutputState>();
    }

    // Update is called once per frame
    void Update()
    {
        //controllerButtonState
        //press
        OutputState.outputState.controllerButtonState.leftController.press[0] = OVRInput.Get(OVRInput.Button.PrimaryIndexTrigger) || Input.GetKey("space");
        OutputState.outputState.controllerButtonState.leftController.press[1] = OVRInput.Get(OVRInput.Button.Three);
        OutputState.outputState.controllerButtonState.leftController.press[2] = OVRInput.Get(OVRInput.Button.Four);
        OutputState.outputState.controllerButtonState.leftController.press[3] = OVRInput.Get(OVRInput.Button.PrimaryHandTrigger);
        OutputState.outputState.controllerButtonState.rightController.press[0] = OVRInput.Get(OVRInput.Button.SecondaryIndexTrigger);
        OutputState.outputState.controllerButtonState.rightController.press[1] = OVRInput.Get(OVRInput.Button.One);
        OutputState.outputState.controllerButtonState.rightController.press[2] = OVRInput.Get(OVRInput.Button.Two);
        OutputState.outputState.controllerButtonState.rightController.press[3] = OVRInput.Get(OVRInput.Button.SecondaryHandTrigger);
        //touch
        OutputState.outputState.controllerButtonState.leftController.touch[0] = OVRInput.Get(OVRInput.Touch.PrimaryIndexTrigger);
        OutputState.outputState.controllerButtonState.leftController.touch[1] = OVRInput.Get(OVRInput.Touch.Three);
        OutputState.outputState.controllerButtonState.leftController.touch[2] = OVRInput.Get(OVRInput.Touch.Four);
        OutputState.outputState.controllerButtonState.leftController.touch[3] = OVRInput.Get(OVRInput.Touch.PrimaryThumbRest);
        OutputState.outputState.controllerButtonState.leftController.touch[4] = OVRInput.Get(OVRInput.Touch.PrimaryThumbstick);
        OutputState.outputState.controllerButtonState.leftController.touch[5] = OVRInput.Get(OVRInput.Touch.PrimaryTouchpad);
        OutputState.outputState.controllerButtonState.rightController .touch[0] = OVRInput.Get(OVRInput.Touch.SecondaryIndexTrigger);
        OutputState.outputState.controllerButtonState.rightController .touch[1] = OVRInput.Get(OVRInput.Touch.One);
        OutputState.outputState.controllerButtonState.rightController .touch[2] = OVRInput.Get(OVRInput.Touch.Two);
        OutputState.outputState.controllerButtonState.rightController .touch[3] = OVRInput.Get(OVRInput.Touch.SecondaryThumbRest);
        OutputState.outputState.controllerButtonState.rightController .touch[4] = OVRInput.Get(OVRInput.Touch.SecondaryThumbstick);
        OutputState.outputState.controllerButtonState.rightController .touch[5] = OVRInput.Get(OVRInput.Touch.SecondaryTouchpad);
        //nearTouch
        OutputState.outputState.controllerButtonState.leftController.nearTouch[0] = OVRInput.Get(OVRInput.NearTouch.PrimaryIndexTrigger);
        OutputState.outputState.controllerButtonState.leftController.nearTouch[1] = OVRInput.Get(OVRInput.NearTouch.PrimaryThumbButtons);
        OutputState.outputState.controllerButtonState.rightController.nearTouch[0] = OVRInput.Get(OVRInput.NearTouch.SecondaryIndexTrigger);
        OutputState.outputState.controllerButtonState.rightController.nearTouch[1] = OVRInput.Get(OVRInput.NearTouch.SecondaryThumbButtons);
        //squeeze
        OutputState.outputState.controllerButtonState.leftController.squeeze[0] = OVRInput.Get(OVRInput.Axis1D.PrimaryIndexTrigger);
        OutputState.outputState.controllerButtonState.leftController.squeeze[1] = OVRInput.Get(OVRInput.Axis1D.PrimaryHandTrigger);
        OutputState.outputState.controllerButtonState.rightController.squeeze[0] = OVRInput.Get(OVRInput.Axis1D.SecondaryIndexTrigger);
        OutputState.outputState.controllerButtonState.rightController.squeeze[1] = OVRInput.Get(OVRInput.Axis1D.SecondaryHandTrigger);
        //thumbstickMovement
        OutputState.outputState.controllerButtonState.leftController.thumbstickMovement[0] = OVRInput.Get(OVRInput.Axis2D.PrimaryThumbstick)[0];
        OutputState.outputState.controllerButtonState.leftController.thumbstickMovement[1] = OVRInput.Get(OVRInput.Axis2D.PrimaryThumbstick)[1];
        OutputState.outputState.controllerButtonState.rightController.thumbstickMovement[0] = OVRInput.Get(OVRInput.Axis2D.SecondaryThumbstick)[0];
        OutputState.outputState.controllerButtonState.rightController.thumbstickMovement[1] = OVRInput.Get(OVRInput.Axis2D.SecondaryThumbstick)[1];



        //controllerPositionState
        //position
        OutputState.outputState.controllerPositionState.leftController.controllerPosition[0] = left.pos.x;
        OutputState.outputState.controllerPositionState.leftController.controllerPosition[1] = left.pos.y;
        OutputState.outputState.controllerPositionState.leftController.controllerPosition[2] = left.pos.z;
        OutputState.outputState.controllerPositionState.rightController.controllerPosition[0] = right.pos.x;
        OutputState.outputState.controllerPositionState.rightController.controllerPosition[1] = right.pos.y;
        OutputState.outputState.controllerPositionState.rightController.controllerPosition[2] = right.pos.z;
        //rotation
        OutputState.outputState.controllerPositionState.leftController.controllerOrientation[0] = left.rot.x;
        OutputState.outputState.controllerPositionState.leftController.controllerOrientation[1] = left.rot.y;
        OutputState.outputState.controllerPositionState.leftController.controllerOrientation[2] = left.rot.z;
        OutputState.outputState.controllerPositionState.leftController.controllerOrientation[3] = left.rot.w;
        OutputState.outputState.controllerPositionState.rightController.controllerOrientation[0] = right.rot.x;
        OutputState.outputState.controllerPositionState.rightController.controllerOrientation[1] = right.rot.y;
        OutputState.outputState.controllerPositionState.rightController.controllerOrientation[2] = right.rot.z;
        OutputState.outputState.controllerPositionState.rightController.controllerOrientation[3] = right.rot.w;



        //headSetPositionState
        OutputState.outputState.headSetPositionState.deviceRotation[0] = headset.centerEyeAnchor.rotation.x;
        OutputState.outputState.headSetPositionState.deviceRotation[1] = headset.centerEyeAnchor.rotation.y;
        OutputState.outputState.headSetPositionState.deviceRotation[2] = headset.centerEyeAnchor.rotation.z;
        OutputState.outputState.headSetPositionState.deviceRotation[3] = headset.centerEyeAnchor.rotation.w;
    }
}
