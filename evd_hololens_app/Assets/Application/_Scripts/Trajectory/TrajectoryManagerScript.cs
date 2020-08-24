using System.Collections;
using System.Collections.Generic;
using UnityEngine;


public class TrajectoryManagerScript : MonoBehaviour
{
    public GameObject TrajectoryPrefab;

    public InteractiveMarkerManagerScript markerManager;

    private System.Action<string, Vector3, Quaternion, int> _colliderClickCallback = null;

    
    private Dictionary<string, List<string>> markerTrajectoryMap = new Dictionary<string, List<string>>();
    private Dictionary<string, TrajectoryRenderScript> trajectories = new Dictionary<string, TrajectoryRenderScript>();


    private void Update()
    {
        foreach (var entry in markerTrajectoryMap)
        {
            if (markerManager.MarkerBeingManipulated(entry.Key))
            {
                foreach (var t in entry.Value)
                {
                    trajectories[t].OnManipulatedWaypoint(entry.Key);
                }
            }
        }
    }

    public System.Action<string, Vector3, Quaternion, int> colliderClickCallback
    {
        private get
        {
            return _colliderClickCallback;
        }

        set
        {
            if (_colliderClickCallback != value)
            {
                _colliderClickCallback = value;
                foreach (var entry in trajectories)
                {
                    entry.Value.colliderClickCallback = _colliderClickCallback;
                }
            }
        }
    }

    public TrajectoryRenderScript GetTrajectory(string uuid)
    {
        if (trajectories.ContainsKey(uuid))
        {
            return trajectories[uuid];
        }
        else
        {
            return null;
        }
    }

    public void OnTrajectoryCreate(Cobots.Trajectory trajectory)
    {
        var obj = Instantiate(TrajectoryPrefab, transform, false);
        var ts = obj.GetComponent<TrajectoryRenderScript>();


        var start = markerManager.GetMarker(trajectory.startLocationUuid).GetComponent<TrajectoryPointScript>();
        AddTrajectoryToMarkerMap(trajectory.startLocationUuid, trajectory.uuid);
        var end = markerManager.GetMarker(trajectory.endLocationUuid).GetComponent<TrajectoryPointScript>();
        AddTrajectoryToMarkerMap(trajectory.endLocationUuid, trajectory.uuid);

        var ways = new List<TrajectoryPointScript>();
        foreach(var w in trajectory.waypoints)
        {
            ways.Add(markerManager.GetMarker(w.uuid).GetComponent<TrajectoryPointScript>());
            AddTrajectoryToMarkerMap(w.uuid, trajectory.uuid);
        }

        ts.OnCreate(start, end, ways);
        ts.colliderClickCallback = colliderClickCallback;

        trajectories.Add(trajectory.uuid, ts);
    }

    public void OnAddWaypoint(Cobots.Trajectory trajectory, string waypointUuid)
    {
        if (trajectories.ContainsKey(trajectory.uuid))
        {
            var wp = markerManager.GetMarker(waypointUuid).GetComponent<TrajectoryPointScript>();
            var idx = trajectory.SearchWaypointsForIndex(waypointUuid);
            trajectories[trajectory.uuid].OnAddWaypoint(wp, idx);
            AddTrajectoryToMarkerMap(waypointUuid, trajectory.uuid);
        }
    }

    public void OnReorderWaypoints(string uuid)
    {
        if (trajectories.ContainsKey(uuid))
        {
            trajectories[uuid].OnReorderWaypoints();
        }
    }

    public void OnDeleteWaypoint(Cobots.Trajectory trajectory, string waypointUuid)
    {
        if (trajectories.ContainsKey(trajectory.uuid))
        {
            trajectories[trajectory.uuid].OnDeleteWaypoint(waypointUuid);
            RemoveTrajectoryFromMarkerMap(waypointUuid, trajectory.uuid);
        }
    }

    public void OnTrajectoryDelete(string uuid)
    {
        if (trajectories.ContainsKey(uuid))
        {
            trajectories[uuid].OnDelete();
            Destroy(trajectories[uuid].gameObject);
            trajectories.Remove(uuid);

            foreach (var entry in markerTrajectoryMap)
            {
                if (entry.Value.Contains(uuid))
                {
                    entry.Value.Remove(uuid);
                }
            }
        }
    }

    private void AddTrajectoryToMarkerMap(string markerUuid, string trajectoryUuid)
    {
        if (!markerTrajectoryMap.ContainsKey(markerUuid))
        {
            markerTrajectoryMap.Add(markerUuid, new List<string>());
        }


        if (!markerTrajectoryMap[markerUuid].Contains(trajectoryUuid))
        {
            markerTrajectoryMap[markerUuid].Add(trajectoryUuid);
        }
    }

    private void RemoveTrajectoryFromMarkerMap(string markerUuid, string trajectoryUuid)
    {
        if (markerTrajectoryMap.ContainsKey(markerUuid) && markerTrajectoryMap[markerUuid].Contains(trajectoryUuid))
        {
            markerTrajectoryMap[markerUuid].Remove(trajectoryUuid);
        }
    }
}
