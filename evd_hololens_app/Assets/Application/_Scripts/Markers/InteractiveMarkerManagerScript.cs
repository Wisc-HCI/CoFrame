﻿using System.Collections;
using System.Collections.Generic;
using UnityEngine;


public class InteractiveMarkerManagerScript : MonoBehaviour
{

    public GameObject InteractiveMarkerPrefab;
    public GameObject NonInteractivePrefab;

    private System.Action<Cobots.AbstractMarker, bool> _markerSelectedCallback = null; 

    private Dictionary<string, Cobots.AbstractMarker> markers = new Dictionary<string, Cobots.AbstractMarker>();

    public System.Action<Cobots.AbstractMarker, bool> markerSelectedCallback
    {
        private get
        {
            return _markerSelectedCallback;
        }

        set
        {
            if (_markerSelectedCallback != value)
            {
                _markerSelectedCallback = value;
                foreach (var entry in markers)
                {
                    entry.Value.selectedCallback = _markerSelectedCallback;
                }
            }
        }
    }

    public bool MarkerBeingManipulated(string uuid)
    {
        if (markers.ContainsKey(uuid))
        {
            return markers[uuid].isBeingManipulated;
        }
        else
        {
            return false;
        }
    }

    public void SetVisibile(string uuid, bool val)
    {
        if (markers.ContainsKey(uuid))
        {
            markers[uuid].visibility = val;
        }
    }

    public bool GetVisible(string uuid)
    {
        if (markers.ContainsKey(uuid))
        {
            return markers[uuid].visibility;
        } 
        else
        {
            return false;
        }
    }

    public Cobots.AbstractMarker GetMarker(string uuid)
    {
       if (markers.ContainsKey(uuid))
       {
            return markers[uuid];
       }
       else
       {
            return null;
       }
    }

    public void OnWaypointUpdate(Cobots.Waypoint way)
    {
        var ms = markers[way.uuid];
        if (!ms.isBeingManipulated)
        {
            ms.Position = way.position.ToUnity();
            ms.Orientation = way.orientation.ToUnity();
        }
        ms.name = way.name;
    }

    public void OnWaypointAdd(Cobots.Waypoint way)
    {
        var obj = Instantiate(InteractiveMarkerPrefab, transform, false);
        var ms = obj.GetComponent<Cobots.AbstractMarker>();
        ms.waypoint = way;
        ms.selectedCallback = markerSelectedCallback;

        markers.Add(way.uuid, ms);
    }

    public void OnWaypointDelete(string uuid)
    {
        if (markers.ContainsKey(uuid))
        {
            Destroy(markers[uuid].gameObject);
            markers.Remove(uuid);
        }
    }
}