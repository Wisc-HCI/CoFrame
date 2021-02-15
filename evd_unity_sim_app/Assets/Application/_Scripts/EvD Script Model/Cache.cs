using System.Collections;
using System.Collections.Generic;

using EvD.Data;
using EvD.Environment;


namespace EvD
{
    class Cache
    {
        /*
         * Static Attributes
         */

        public static Dictionary<string,Node> data = new Dictionary<string, Node>();
        public static Dictionary<string,Trajectory> trajectories = new Dictionary<string, Trajectory>();
        public static Dictionary<string,Location> locations = new Dictionary<string, Location>();
        public static Dictionary<string,Waypoint> waypoints = new Dictionary<string, Waypoint>();
        public static Dictionary<string,Thing> things = new Dictionary<string, Thing>();
        public static Dictionary<string,Trace> traces = new Dictionary<string, Trace>();
        public static Dictionary<string,EnvironmentContext> environments = new Dictionary<string, EnvironmentContext>();

        /*
         * Static Methods
         */
        public static void Add(string uuid, Node node) 
        {
            data.Add(uuid,node);

            if (node is Trajectory) 
            {
                trajectories.Add(uuid,(Trajectory)node);
            }

            if (node is Location) 
            {
                locations.Add(uuid, (Location)node);
            }

            if (node is Waypoint)
            {
                waypoints.Add(uuid, (Waypoint)node);
            }

            if (node is Thing)
            {
                things.Add(uuid, (Thing)node);
            }

            if (node is Trace)
            {
                traces.Add(uuid, (Trace)node);
            }

            if (node is EnvironmentContext)
            {
                environments.Add(uuid, (EnvironmentContext)node);
            }
        }

        public static void Remove(string uuid) 
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

            if (node is Thing)
            {
                things.Remove(uuid);
            }

            if (node is Trace)
            {
                traces.Remove(uuid);
            }

            if (node is EnvironmentContext)
            {
                environments.Remove(uuid);
            }

            data.Remove(uuid);
        }

        public static void Clear() 
        {
            data.Clear();
            trajectories.Clear();
            locations.Clear();
            waypoints.Clear();
            things.Clear();
            traces.Clear();
            environments.Clear();
        }

        public static Node Get(string uuid, string hint = null)
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
            else if (hint == "thing" && things.ContainsKey(uuid)) 
            {
                retNode = things[uuid];
            }
            else if (hint == "trace" && traces.ContainsKey(uuid))
            {
                retNode = traces[uuid];
            }
            else if (hint == "environment" && environments.ContainsKey(uuid))
            {
                retNode = environments[uuid];
            }
            else
            {
                retNode = data[uuid];
            }

            return retNode;
        }

        public static void Set(string uuid, Dictionary<string, object> dct, string hint=null)
        {
            Get(uuid,hint).Set(dct);
        }
    }
}