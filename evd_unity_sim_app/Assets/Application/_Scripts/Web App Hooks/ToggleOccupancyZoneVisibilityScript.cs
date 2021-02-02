using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ToggleOccupancyZoneVisibilityScript : MonoBehaviour
{

    public GameObject OccupancyZoneManager = null;   


    public void ToggleVisibility(bool state)
    {
        //TODO Command the manager to toggle the occupancy zones
        Debug.Log(state);
    }
}
