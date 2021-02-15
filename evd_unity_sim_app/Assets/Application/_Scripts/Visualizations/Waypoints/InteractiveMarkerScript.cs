using System.Collections;
using System.Collections.Generic;
using UnityEngine;


public class InteractiveMarkerScript : AbstractMarker
{
    

    public void OnManipulationStart()
    {
        isBeingManipulated = true;
        selectedCallback?.Invoke(this, true);
    }

    public void OnManipulationEnd()
    {
        isBeingManipulated = false;
        selectedCallback?.Invoke(this, false);
    }

    private void Update()
    {
        if (waypoint != null && isBeingManipulated)
        {
            waypoint.position = EvD.Data.Position.FromUnity(Position);
            waypoint.orientation = EvD.Data.Orientation.FromUnity(Orientation);
        }
    }
}
