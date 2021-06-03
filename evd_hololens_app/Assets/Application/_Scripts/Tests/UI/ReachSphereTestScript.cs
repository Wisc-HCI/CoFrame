using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class ReachSphereTestScript : MonoBehaviour
{
    public ReachSphereScript reachSphere;

    public bool state = false;

    public Text actionButtonText;
    public Dropdown colorDropdown;

    private void Awake()
    {
        actionButtonText.text = (state) ? "Collapse" : "Expand"; 
    }

    public void OnButtonPress()
    {
        state = !state;
        reachSphere.expand = state;
        actionButtonText.text = (state) ? "Collapse" : "Expand";
    }

    public void OnValueChange()
    {
        reachSphere.colorState = (ReachSphereScript.ColorState)(colorDropdown.value);
    }
}
