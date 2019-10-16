using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class OVRInputMap : MonoBehaviour
{
    private OutputState OutputState;
    public OVRGrabber left;
    public OVRGrabber right;
    void Start()
    {
        OutputState = gameObject.GetComponent<OutputState>();
    }

    // Update is called once per frame
    void Update()
    {
        Debug.Log(Input.GetKey("space"));
        OutputState.outputState.controllerButtonState.leftController.press[0] = OVRInput.Get(OVRInput.Button.PrimaryIndexTrigger) || Input.GetKey("space");
        OutputState.outputState.controllerButtonState.leftController.press[1] = OVRInput.Get(OVRInput.Button.Three);
        OutputState.outputState.controllerButtonState.leftController.press[2] = OVRInput.Get(OVRInput.Button.Four);
        OutputState.outputState.controllerButtonState.leftController.press[3] = OVRInput.Get(OVRInput.Button.PrimaryHandTrigger);
        OutputState.outputState.controllerButtonState.rightController.press[0] = OVRInput.Get(OVRInput.Button.SecondaryIndexTrigger);
        OutputState.outputState.controllerButtonState.rightController.press[0] = OVRInput.Get(OVRInput.Button.One);
        OutputState.outputState.controllerButtonState.rightController.press[0] = OVRInput.Get(OVRInput.Button.Two);
        OutputState.outputState.controllerButtonState.rightController.press[0] = OVRInput.Get(OVRInput.Button.SecondaryHandTrigger);
        OutputState.outputState.controllerPositionState.leftController.controllerPosition[0] = left.pos.x;
        OutputState.outputState.controllerPositionState.leftController.controllerPosition[1] = left.pos.y;
        OutputState.outputState.controllerPositionState.leftController.controllerPosition[2] = left.pos.z;
        OutputState.outputState.controllerPositionState.rightController.controllerPosition[0] = right.pos.x;
        OutputState.outputState.controllerPositionState.rightController.controllerPosition[1] = right.pos.y;
        OutputState.outputState.controllerPositionState.rightController.controllerPosition[2] = right.pos.z;
    }
}
