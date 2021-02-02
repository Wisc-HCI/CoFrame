using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ToggleReachSphereScript : MonoBehaviour
{
    public GameObject ReachSphereManager = null;   


    public void ToggleVisibility(bool state)
    {
        // TODO command the manager to toggle the reach sphere
        Debug.Log(state);
    }
}
