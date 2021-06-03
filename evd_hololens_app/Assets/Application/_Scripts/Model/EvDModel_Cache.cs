using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace EvD
{
    [System.Serializable]
    public class Cache
    {

        public Dictionary<string, Node> data = new Dictionary<string, Node>();
        public Dictionary<string, Trajectory> trajectories = new Dictionary<string, Trajectory>();
        public Dictionary<string, Location> locations = new Dictionary<string, Location>();
        public Dictionary<string, Waypoint> waypoints = new Dictionary<string, Waypoint>();

        public void Add(string uuid, Node node)
        {
            data.Add(uuid, node);

            if (node is Trajectory)
            {
                trajectories.Add(uuid, (Trajectory)node);
            }
            
            if (node is Location)
            {
                locations.Add(uuid, (Location)node);
            }

            if (node is Waypoint)
            {
                waypoints.Add(uuid, (Waypoint)node);
            }
        }

        public void Remove(string uuid)
        {
            var node = data[uuid];

            if (node is Trajectory)
            {
                trajectories.Remove(uuid);
            }

            if (node is Location)
            {
                locations.Remove(uuid);
            }

            if (node is Waypoint) 
            {
                waypoints.Remove(uuid);
            }

            data.Remove(uuid); 
        }

        public void Clear()
        {
            data.Clear();
            trajectories.Clear();
            locations.Clear();
        }

        public Node Get(string uuid, string hint = null)
        {
            Node retNode = null;

            if (hint == "trajectory" && trajectories.ContainsKey(uuid))
            {
                retNode = trajectories[uuid];
            }
            else if (hint == "location" && locations.ContainsKey(uuid))
            {
                retNode = locations[uuid];
            }
            else if (hint == "waypoint" && waypoints.ContainsKey(uuid))
            {
                retNode = waypoints[uuid];
            }
            else
            {
                retNode = data[uuid];
            }

            return retNode;
        }

        public void Set(string uuid, Dictionary<string,object> dct, string hint=null) 
        {
            Get(uuid,hint).Set(dct);
        }

    }
}
