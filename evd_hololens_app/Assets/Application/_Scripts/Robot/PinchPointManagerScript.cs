using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PinchPointManagerScript : MonoBehaviour
{
    [System.Serializable]
    public class PinchPointEntry
    {
        public string name;
        public PinchPointScript script;
    }

    public List<PinchPointEntry> pinchPoints = new List<PinchPointEntry>();

    public void SetVisible(string name, bool val)
    {
        foreach (var p in pinchPoints)
        {
            if (name == p.name)
            {
                p.script.visible = val;
            }
        }
    }

    public bool GetVisible(string name)
    {
        bool val = false;
        foreach (var p in pinchPoints)
        {
            if (name == p.name)
            {
                val = p.script.visible;
            }
        }
        return val;
    }

    public void OnPinchPointUpdate(Cobots.PinchPoint pState)
    {
        foreach (var p in pinchPoints)
        {
            if (pState.name == p.name)
            {
                if (pState.state == "good")
                {
                    p.script.colorState = PinchPointScript.ColorState.Good;
                }
                else if (pState.state == "warn")
                {
                    p.script.colorState = PinchPointScript.ColorState.Warn;
                }
                else if (pState.state == "error")
                {
                    p.script.colorState = PinchPointScript.ColorState.Error;
                }
            }
        }
    }
}
