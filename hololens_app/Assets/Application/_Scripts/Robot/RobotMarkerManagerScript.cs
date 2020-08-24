using System.Collections.Generic;
using UnityEngine;

public class RobotMarkerManagerScript : MonoBehaviour
{
    //TODO opacity management algorithm - Danny's paper

    public GameObject MarkerPrefab;

    private Dictionary<string, RobotMarkerJointStateWriterScript> activeMarkers = new Dictionary<string, RobotMarkerJointStateWriterScript>();
    private Dictionary<string, RobotMarkerJointStateWriterScript> disabledeMarkers = new Dictionary<string, RobotMarkerJointStateWriterScript>();


    public void SetVisible(string uuid, bool val)
    {
        if (activeMarkers.ContainsKey(uuid))
        {
            activeMarkers[uuid].visibility = val;
        }
    }

    public bool GetVisible(string uuid)
    {
        if (activeMarkers.ContainsKey(uuid))
        {
            return activeMarkers[uuid].visibility;
        }
        else
        {
            return false;
        }
    }

    public void OnWaypointUpdate(Cobots.Waypoint way)
    {
        bool hasJoints = way.joints != null;
        bool prevJoints = activeMarkers.ContainsKey(way.uuid);

        if (hasJoints && !prevJoints)
        {
            var ms = disabledeMarkers[way.uuid];
            disabledeMarkers.Remove(way.uuid);
            activeMarkers.Add(way.uuid,ms);

            ms.SetJointStates(way.joints);
            ms.visibility = true;
        }
        else if (hasJoints && prevJoints)
        {
            activeMarkers[way.uuid].SetJointStates(way.joints);
        }
        else if (!hasJoints && prevJoints)
        {
            var ms = activeMarkers[way.uuid];
            activeMarkers.Remove(way.uuid);
            disabledeMarkers.Add(way.uuid, ms);
            ms.visibility = false;
        }
    }

    public void OnWaypointAdd(Cobots.Waypoint way)
    {
        var obj = Instantiate(MarkerPrefab, transform, false);
        var ms = obj.GetComponent<RobotMarkerJointStateWriterScript>();

        bool initJoints = way.joints != null;
        ms.visibility = initJoints;
        if (initJoints)
        {
            ms.SetJointStates(way.joints);
            ms.SetOpacities(new float[way.joints.Length]);
        }

        if (initJoints)
        {
            activeMarkers.Add(way.uuid, ms);
        }
        else
        {
            disabledeMarkers.Add(way.uuid, ms);
        }
    }

    public void OnWaypointDelete(string uuid)
    {
        if (activeMarkers.ContainsKey(uuid))
        {
            Destroy(activeMarkers[uuid].gameObject);
            activeMarkers.Remove(uuid);
        }
        else
        {
            Destroy(disabledeMarkers[uuid].gameObject);
            disabledeMarkers.Remove(uuid);
        }
    }

}
