using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MarkerMeshTypeScript : MonoBehaviour
{
    public GameObject locationMarker = null;
    public GameObject waypointMarker = null;

    public enum Options
    {
        Location,
        Waypoint
    }

    public Options type = Options.Location;

    private void Start()
    {
        if (type == Options.Location)
        {
            locationMarker.SetActive(true);
            waypointMarker.SetActive(false);
        }
        else if (type == Options.Waypoint)
        {
            locationMarker.SetActive(false);
            waypointMarker.SetActive(true);
        }
    }

    public void SetMeshType(Options t)
    {
        type = t;

        if (type == Options.Location)
        {
            locationMarker.SetActive(true);
            waypointMarker.SetActive(false);
        }
        else if (type == Options.Waypoint)
        {
            locationMarker.SetActive(false);
            waypointMarker.SetActive(true);
        }
    }
}
