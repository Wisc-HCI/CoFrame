using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ToggleOccupancyZoneVisibilityScript : MonoBehaviour
{

    public OccupancyZoneManagerScript OccupancyZoneManager = null;   

    public void ToggleVisibility(bool state)
    {
        OccupancyZoneManager.SetVisible(state);
    }

    public void ToggleVisibilitySpecific(string uuid) 
    {
        var state = !OccupancyZoneManager.GetVisible(uuid);
        OccupancyZoneManager.SetVisible(uuid,state);
    }
}
