using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class OVRInputMap : MonoBehaviour
{
    private OutputState OutputState;
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
    }
}
