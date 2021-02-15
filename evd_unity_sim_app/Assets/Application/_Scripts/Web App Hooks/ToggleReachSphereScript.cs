using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ToggleReachSphereScript : MonoBehaviour
{
    public ReachSphereScript ReachSphereManager = null;   


    public void ToggleVisibility(bool state)
    {
        ReachSphereManager.expand = state;
    }
}
