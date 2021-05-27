using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Microsoft.MixedReality.Toolkit.UI;


public class InteractiveMarkerScript : EvD.AbstractMarker
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
            waypoint.position = EvD.Position.FromUnity(Position);
            waypoint.orientation = EvD.Orientation.FromUnity(Orientation);
        }
    }
}
