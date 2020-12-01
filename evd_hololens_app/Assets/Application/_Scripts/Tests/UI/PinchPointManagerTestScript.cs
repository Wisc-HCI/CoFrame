using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using EvD;

public class PinchPointManagerTestScript : MonoBehaviour
{
    public PinchPointManagerScript pinchPointManager;

    public string[] pointNames = new string[4];
    public Toggle[] visibilityToggle = new Toggle[4];
    public Dropdown[] stateDropdowns = new Dropdown[4];

    private void Start()
    {
        for(int i=0; i<4; i++)
        {
            //Debug.Log(pointNames[i]);
            OnVisibleToggle(i);
            OnDropdownChange(i);
        }
    }

    public void OnVisibleToggle(int idx)
    {
        pinchPointManager.SetVisible(pointNames[idx], visibilityToggle[idx].isOn);
    }

    public void OnDropdownChange(int idx)
    {
        string state = null;
        switch (stateDropdowns[idx].value)
        {
            case 0:
                state = PinchPoint.GOOD_STATE;
                break;
            case 1:
                state = PinchPoint.WARN_STATE;
                break;
            case 2:
                state = PinchPoint.ERROR_STATE;
                break;
        }

        PinchPoint p = new PinchPoint(state: state, name: pointNames[idx]);
        pinchPointManager.OnPinchPointUpdate(p);
    }
}
