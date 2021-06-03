using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.PlayerLoop;

namespace EvD
{
    [System.Serializable]
    public class Waypoint : Pose
    {
        /*
         * Private Members
         */

        private double[] _joints;

        /*
         * Constructors
         */

        public Waypoint(Position position = null, Orientation orientation = null, double[] joints = null, 
                        string type = "", string name = "", string uuid = null, Node parent = null, 
                        bool appendType = true) 
        : base(position,orientation, appendType ? "waypoint." + type : type, name,uuid,parent,appendType)
        {
            this.joints = joints;
        }

        public new static Waypoint FromDict(Dictionary<string,object> dct)
        {
            return new Waypoint(
                position: Position.FromDict((Dictionary<string, object>)dct["position"]),
                orientation: Orientation.FromDict((Dictionary<string, object>)dct["orientation"]),
                joints: (double[])dct["joints"],
                type: (string)dct["type"],
                name: (string)dct["name"],
                uuid: (string)dct["uuid"],
                appendType: false
            );
        }

        public override Dictionary<string, object> ToDict()
        {
            var dct = base.ToDict();
            dct.Add("joints", joints);
            return dct;
        }

        /*
         * Accessors and Modifiers
         */

        public double[] joints
        {
            get
            {
                return _joints;
            }

            set
            {
                if (_joints != value)
                {
                    _joints = value;
                    UpdatedAttribute("joints", "set");
                }
            }
        }

        public override void Set(Dictionary<string, object> dct)
        {
            if (dct.ContainsKey("joints"))
            {
                joints = (double[])dct["joints"];
            }

            base.Set(dct);
        }

        /*
         * Update Methods
         */

        public override void DeepUpdate()
        {

            base.DeepUpdate();

            UpdatedAttribute("joints", "update");
        }

        public override void ShallowUpdate()
        {
            base.ShallowUpdate();

            UpdatedAttribute("joints", "update");
        }
    }

    [System.Serializable]
    public class Location : Waypoint
    {

        /*
         * Constructors
         */

        public Location(Position position = null, Orientation orientation = null, double[] joints = null, 
                        string type="", string name = "", string uuid = null, Node parent = null, 
                        bool appendType=true) 
        : base(position,orientation,joints, appendType ? "location." + type : type, name, uuid,parent,appendType) { }

        public new static Location FromDict(Dictionary<string, object> dct)
        {
            return new Location(
                position: Position.FromDict((Dictionary<string, object>)dct["position"]),
                orientation: Orientation.FromDict((Dictionary<string, object>)dct["orientation"]),
                joints: (double[])dct["joints"],
                type: (string)dct["type"],
                name: (string)dct["name"],
                uuid: (string)dct["uuid"],
                appendType: false
            );
        }
    }

    [System.Serializable]
    public class Machine : Node
    {

        /*
         * Constructors
         */

        public Machine(string type = "", string name = "", string uuid = null, 
                       Node parent = null, bool appendType = true) 
        : base(appendType ? "machine." + type : type, name, uuid,parent,appendType) { }

        public new static Machine FromDict(Dictionary<string, object> dct)
        {
            return new Machine(
                type: (string)dct["type"],
                name: (string)dct["name"],
                uuid: (string)dct["uuid"],
                appendType: false
            );
        }
    }

    [System.Serializable]
    public class Trajectory : Node
    {
        public static readonly string[] TYPES = { "joint", "linear", "planner" };

        /*
         * Private Members
         */

        private string _moveType = null;
        private float _velocity = 0;
        private float _acceleration = 0;
        private string _startLocationUuid = null;
        private string _endLocationUuid = null;
        private Trace _trace = null;
        private List<Waypoint> _waypoints = new List<Waypoint>();

        /*
         * Constructors
         */

        public Trajectory(string startLocUuid = null, string endLocUuid = null, List<Waypoint> waypoints = null, 
                          Trace trace = null, string moveType = "joint", float velocity = 0, float acceleration = 0, 
                          string type = "", string name = "", string uuid = null, Node parent = null, 
                          bool appendType = true) 
        : base(appendType ? "trajectory." + type : type, name,uuid,parent,appendType)
        {
            this.moveType = moveType;
            this.velocity = velocity;
            this.acceleration = acceleration;

            this.startLocationUuid = startLocUuid;
            this.endLocationUuid = endLocUuid;

            if (waypoints != null)
            {
                this.waypoints = waypoints;
            }
            
            this.trace = trace;
        }

        public static new Trajectory FromDict(Dictionary<string,object> dct)
        {
            var waypoints = new List<Waypoint>();
            foreach (var wp in (List<Dictionary<string, object>>)dct["waypoints"])
            {
                waypoints.Add(Waypoint.FromDict(wp));
            }

            Trace trace = null;
            if (dct["trace"] != null)
            {
                trace = Trace.FromDict((Dictionary<string, object>)dct["trace"]);
            }

            return new Trajectory(
                startLocUuid: (string)dct["start_location_uuid"],
                endLocUuid: (string)dct["end_location_uuid"],
                moveType: (string)dct["move_type"],
                velocity: (float)dct["velocity"],
                acceleration: (float)dct["acceleration"],
                waypoints: waypoints,
                trace: trace,
                type: (string)dct["type"],
                name: (string)dct["name"],
                uuid: (string)dct["uuid"],
                appendType: false
            );
        }

        public override Dictionary<string,object> ToDict()
        {
            var dctWaypoints = new List<Dictionary<string, object>>();
            foreach (var wp in waypoints)
            {
                dctWaypoints.Add(wp.ToDict());
            }

            Dictionary<string, object> dctTrace = null;
            if (trace != null)
            {
                dctTrace = trace.ToDict();
            }

            var dct = base.ToDict();
            dct.Add("start_location_uuid", startLocationUuid);
            dct.Add("end_location_uuid", endLocationUuid);
            dct.Add("move_type", moveType);
            dct.Add("velocity", velocity);
            dct.Add("acceleration", acceleration);
            dct.Add("waypoints", dctWaypoints);
            dct.Add("trace", dctTrace);
            return dct;
        }

        /*
         * Accessors and Modifiers
         */

        public string moveType
        {
            get
            {
                return _moveType;
            }

            set
            {
                bool found = false;
                foreach (var t in TYPES)
                {
                    if (t == value)
                    {
                        found = true;
                        break;
                    }
                }

                if (!found)
                {
                    throw new System.Exception("Type provided is invalid");
                }

                if (value != _moveType)
                {
                    _moveType = value;
                    trace = null;
                    UpdatedAttribute("move_type", "set");
                }
            }
        }

        public float velocity
        {
            get
            {
                return _velocity;
            }

            set
            {
                if (_velocity != value)
                {
                    _velocity = value;
                    trace = null;
                    UpdatedAttribute("velocity", "set");
                }
            }
        }

        public float acceleration
        {
            get
            {
                return _acceleration;
            }

            set
            {
                if (_acceleration != value)
                {
                    _acceleration = value;
                    trace = null;
                    UpdatedAttribute("acceleration", "set");
                }
            }
        }

        public string startLocationUuid
        {
            get
            {
                return _startLocationUuid;
            }

            set
            {
                if (_startLocationUuid != value)
                {
                    _startLocationUuid = value;
                    trace = null;
                    UpdatedAttribute("start_location_uuid", "set");
                }
            }
        }

        public string endLocationUuid
        {
            get
            {
                return _endLocationUuid;
            }

            set
            {
                if (_endLocationUuid != value)
                {
                    _endLocationUuid = value;
                    trace = null;
                    UpdatedAttribute("end_location_uuid", "set");
                }
            }
        }

        public List<Waypoint> waypoints
        {
            get
            {
                return _waypoints;
            }

            set
            {
                if (_waypoints != value)
                {
                    foreach (var w in _waypoints)
                    {
                        w.RemoveFromCache();
                    }

                    _waypoints = value;
                    foreach (var w in _waypoints)
                    {
                        w.parent = this;
                    }

                    trace = null;
                    UpdatedAttribute("waypoints", "set");
                }
            }
        }

        public Trace trace
        {
            get
            {
                return _trace;
            }

            set
            {
                if (_trace != value)
                {
                    if (_trace != null)
                    {
                        _trace.RemoveFromCache();
                    }

                    _trace = value;
                    if (_trace != null)
                    {
                        _trace.parent = this;
                    }

                    UpdatedAttribute("trace", "set");
                }
            }
        }

        public void AddWaypoint(Waypoint wp)
        {
            wp.parent = this;
            waypoints.Add(wp);
            trace = null;
            UpdatedAttribute("waypoints", "add", wp.uuid);
        }

        public void InsertWaypoint(int idx, Waypoint wp)
        {
            wp.parent = this;
            waypoints.Insert(idx, wp);
            trace = null;
            UpdatedAttribute("waypoints", "add", wp.uuid);
        }

        public Waypoint GetWaypoint(string uuid)
        {
            return waypoints[SearchWaypointsForIndex(uuid)];
        }

        public void ReorderWaypoints(string uuid, int shift)
        {
            int idx = SearchWaypointsForIndex(uuid);

            if (idx != -1)
            {
                int shiftedIdx = idx + shift;
                if (shiftedIdx < 0 || shiftedIdx >= waypoints.Count)
                {
                    throw new System.Exception("Index out of bounds");
                }

                var copy = waypoints[idx];
                waypoints.RemoveAt(idx);
                waypoints.Insert(shiftedIdx, copy);
                UpdatedAttribute("waypoints", "reorder");
            }
        }

        public void DeleteWaypoint(string wpId)
        {
            int idx = SearchWaypointsForIndex(wpId);

            if (idx == -1)
            {
                throw new System.Exception("Waypoint not in list");
            }

            waypoints[idx].RemoveFromCache();
            waypoints.RemoveAt(idx);
            trace = null;
            UpdatedAttribute("waypoints", "delete", wpId);
        }

        public int SearchWaypointsForIndex(string uuid)
        {
            int idx = -1;
            for (int i = 0; i < waypoints.Count; i++)
            {
                if (waypoints[i].uuid == uuid)
                {
                    idx = i;
                    break;
                }
            }
            return idx;
        }

        public override void Set(Dictionary<string,object> dct)
        {
            if (dct.ContainsKey("start_location_uuid"))
            {
                startLocationUuid = (string)dct["start_location_uuid"];
            }

            if (dct.ContainsKey("end_location_uuid"))
            {
                endLocationUuid = (string)dct["end_location_uuid"];
            }

            if (dct.ContainsKey("waypoints"))
            {
                var waypoints = new List<Waypoint>();
                foreach (var wp in (List<Dictionary<string, object>>)dct["waypoints"])
                {
                    waypoints.Add(Waypoint.FromDict(wp));
                }
                this.waypoints = waypoints;
            }

            if (dct.ContainsKey("velocity"))
            {
                velocity = (float)dct["velocity"];
            }

            if (dct.ContainsKey("acceleration"))
            {
                acceleration = (float)dct["acceleration"];
            }

            if (dct.ContainsKey("move_type"))
            {
                moveType = (string)dct["move_type"];
            }

            if (dct.ContainsKey("trace"))
            {
                Trace trace = null;
                if (dct["trace"] != null)
                {
                    trace = Trace.FromDict((Dictionary<string, object>)dct["trace"]);
                }
                this.trace = trace;
            }

            base.Set(dct);
        }

        /*
         * Cache Methods
         */

        public override void RemoveFromCache()
        {
            foreach (var wp in waypoints)
            {
                wp.RemoveFromCache();
            }

            if (trace != null)
            {
                trace.RemoveFromCache();
            }

            base.RemoveFromCache();
        } 

        public override void AddToCache()
        {
            foreach (var wp in waypoints)
            {
                wp.AddToCache();
            }

            if (trace != null)
            {
                trace.AddToCache();
            }

            base.AddToCache();
        }

        /*
         * Children Methods
         */

        public override bool DeleteChild(string uuid)
        {
            bool success = false;
            
            if (trace != null && trace.uuid == uuid)
            {
                trace = null;
                success = true;
            }
            else
            {
                foreach (var wp in waypoints)
                {
                    if (wp.uuid == uuid)
                    {
                        DeleteWaypoint(uuid);
                        success = true;
                        break;
                    }
                }
            }

            return success;
        }

        /*
         * Update Methods
         */

        public override void DeepUpdate()
        {
            foreach (var w in waypoints)
            {
                w.DeepUpdate();
            }

            if (trace != null)
            {
                trace.DeepUpdate();
            }

            base.DeepUpdate();

            UpdatedAttribute("start_location_uuid", "update");
            UpdatedAttribute("end_location_uuid", "update");
            UpdatedAttribute("waypoints", "update");
            UpdatedAttribute("velocity", "update");
            UpdatedAttribute("acceleration", "update");
            UpdatedAttribute("move_type", "update");
            UpdatedAttribute("trace", "update");
        }

        public override void ShallowUpdate()
        {
            base.ShallowUpdate();

            UpdatedAttribute("start_location_uuid", "update");
            UpdatedAttribute("end_location_uuid", "update");
            UpdatedAttribute("waypoints", "update");
            UpdatedAttribute("velocity", "update");
            UpdatedAttribute("acceleration", "update");
            UpdatedAttribute("move_type", "update");
            UpdatedAttribute("trace", "update");
        }
    }

    [System.Serializable]
    public class TraceDataPoint : Pose
    {
        /*
         * Private Members
         */

        private float _grade;

        /*
         * Constructors
         */

        public TraceDataPoint(Position position, Orientation orientation, float grade, string type = "", 
                              string name = "", string uuid = null, Node parent = null, bool appendType = true) 
        : base(position, orientation, appendType ? "trace-data-point." + type : type, name, uuid, parent, appendType)
        {
            this.grade = grade;
        }

        public new static TraceDataPoint FromDict(Dictionary<string,object> dct)
        {
            return new TraceDataPoint(
                position: Position.FromDict((Dictionary<string, object>)dct["position"]),
                orientation: Orientation.FromDict((Dictionary<string,object>)dct["orientation"]),
                grade: (float)dct["grade"],
                type: (string)dct["type"],
                name: (string)dct["name"],
                uuid: (string)dct["uuid"],
                appendType: false
            );
        }

        public override Dictionary<string, object> ToDict()
        {
            var dct = base.ToDict();
            dct.Add("grade", grade);
            return dct;
        }

        /*
         * Accessors and Modifiers
         */

        public float grade
        {
            get
            {
                return _grade;
            }

            set
            {
                if (_grade != value)
                {
                    _grade = value;
                    UpdatedAttribute("grade", "set");
                }
            }
        }

        public override void Set(Dictionary<string, object> dct)
        {
            if (dct.ContainsKey("grade"))
            {
                grade = (float)dct["grade"];
            }

            base.Set(dct);
        }

        /*
         * Update Methods
         */

        public override void DeepUpdate()
        {
            base.DeepUpdate();

            UpdatedAttribute("grade", "update");
        }

        public override void ShallowUpdate()
        {
            base.ShallowUpdate();

            UpdatedAttribute("grade", "update");
        }
    }

    [System.Serializable]
    public class Trace : Node
    {
        /*
         * Private Members
         */

        private string _endEffectorPath = null;
        private List<string> _jointPaths = null;
        private List<string> _toolPaths = null;
        private List<string> _componentPaths = null;
        private float _time = 0;
        private Dictionary<string, List<TraceDataPoint>> _data;

        /*
         * Constructors
         */

        public Trace(string eePath, Dictionary<string, List<TraceDataPoint>> data, List<string> jPaths = null,
                     List<string> tPaths = null, List<string> cPaths = null, float time = 0, string type = "", 
                     string name = "",  string uuid = null, Node parent = null, bool appendType = true) 
        : base(appendType ? "trace." + type : type, name, uuid, parent, appendType)
        {
            this.endEffectorPath = eePath;
            this.data = data == null ? new Dictionary<string, List<TraceDataPoint>>() : data;
            this.jointPaths = jPaths == null ? new List<string>() : jPaths;
            this.toolPaths = tPaths == null ? new List<string>() : tPaths;
            this.componentPaths = cPaths == null ? new List<string>() : cPaths;
            this.time = time;
        }

        public new static Trace FromDict(Dictionary<string,object> dct)
        {
            var data = new Dictionary<string, List<TraceDataPoint>>();
            foreach (var entry in (Dictionary<string, List<Dictionary<string, object>>>)dct["data"])
            {
                var l = new List<TraceDataPoint>();
                foreach (var dp in entry.Value)
                {
                    l.Add(TraceDataPoint.FromDict(dp));
                }
                data.Add(entry.Key, l);
            }

            return new Trace(
                eePath: (string)dct["end_effector_path"],
                data: data,
                jPaths: (List<string>)dct["joint_paths"],
                tPaths: (List<string>)dct["tool_paths"],
                cPaths: (List<string>)dct["component_paths"],
                time: (float)dct["time"],
                type: (string)dct["type"],
                name: (string)dct["name"],
                uuid: (string)dct["uuid"],
                appendType: false
            );
        }

        public override Dictionary<string,object> ToDict()
        {
            var dataDct = new Dictionary<string, object>();

            foreach (var entry in data)
            {
                var lDct = new List<Dictionary<string, object>>();
                foreach (var dp in entry.Value)
                {
                    lDct.Add(dp.ToDict());
                }
                dataDct.Add(entry.Key, lDct);
            }

            var dct = base.ToDict();
            dct.Add("time", time);
            dct.Add("end_effector_path", endEffectorPath);
            dct.Add("joint_paths", jointPaths);
            dct.Add("tool_paths", toolPaths);
            dct.Add("component_paths", componentPaths);
            dct.Add("data", dataDct);
            return dct;
        }

        /*
         * Accessors and Modifiers
         */

        public string endEffectorPath
        {
            get
            {
                return _endEffectorPath;
            }

            set
            {
                if (_endEffectorPath != value)
                {
                    _endEffectorPath = value;
                    UpdatedAttribute("end_effector_path", "set");
                }
            }
        }

        public List<string> jointPaths
        {
            get
            {
                return _jointPaths;
            }

            set
            {
                if (_jointPaths != value)
                {
                    _jointPaths = value;
                    UpdatedAttribute("joint_paths", "set");
                }
            }
        }

        public List<string> toolPaths
        {
            get
            {
                return _toolPaths;
            }

            set
            {
                if (_toolPaths != value)
                {
                    _toolPaths = value;
                    UpdatedAttribute("tool_paths", "set");
                }
            }
        }

        public List<string> componentPaths
        {
            get
            {
                return _componentPaths;
            }

            set
            {
                if (_componentPaths != value)
                {
                    _componentPaths = value;
                    UpdatedAttribute("component_paths", "set");
                }
            }
        }

        public float time
        {
            get
            {
                return _time;
            }

            set
            {
                if (_time != value)
                {
                    _time = value;
                    UpdatedAttribute("time", "set");
                }
            }
        }

        public Dictionary<string, List<TraceDataPoint>> data
        {
            get
            {
                return _data;
            }

            set
            {
                if (_data != value)
                {
                    if (_data != null)
                    {
                        foreach (KeyValuePair<string, List<TraceDataPoint>> entry in _data)
                        {
                            foreach (var dp in entry.Value)
                            {
                                dp.RemoveFromCache();
                            }
                        }
                    }
                    
                    _data = value;
                    if (_data != null)
                    {
                        foreach (KeyValuePair<string, List<TraceDataPoint>> entry in _data)
                        {
                            foreach (var dp in entry.Value)
                            {
                                dp.parent = this;
                            }
                        }
                    }
                    
                    UpdatedAttribute("data", "set");
                }
            }
        }

        public void AddDataPoint(string group, TraceDataPoint dp)
        {
            dp.parent = this;
            data[group].Add(dp);
            UpdatedAttribute("data", "add", dp.uuid);
        }

        public TraceDataPoint GetDataPoint(string group, string uuid)
        {
            return data[group][SearchDataForIndex(group,uuid)];
        }

        public void DeleteDataPoint(string group, string uuid)
        {
            int idx = SearchDataForIndex(group, uuid);
            if (idx == -1)
            {
                throw new System.Exception("Datapoint not in trace");
            }

            data[group][idx].RemoveFromCache();
            data[group].RemoveAt(idx);
            UpdatedAttribute("data", "delete", uuid);
        }

        public int SearchDataForIndex(string group, string uuid)
        {
            int idx = -1;
            for (int i = 0; i < data[group].Count; i++)
            {
                if (data[group][i].uuid == uuid)
                {
                    idx = i;
                    break;
                }
            }
            return idx;
        }

        public override void Set(Dictionary<string,object> dct)
        {
            if (dct.ContainsKey("data"))
            {
                var d = new Dictionary<string, List<TraceDataPoint>>();
                foreach (var entry in (Dictionary<string, List<Dictionary<string, object>>>)dct["data"])
                {
                    var l = new List<TraceDataPoint>();
                    foreach (var dp in entry.Value)
                    {
                        l.Add(TraceDataPoint.FromDict(dp));
                    }
                    d.Add(entry.Key, l);
                }

                data = d;
            }

            if (dct.ContainsKey("time"))
            {
                time = (float)dct["time"];
            }

            if (dct.ContainsKey("end_effector_path"))
            {
                endEffectorPath = (string)dct["end_effector_path"];
            }

            if (dct.ContainsKey("joint_paths"))
            {
                jointPaths = (List<string>)dct["joint_paths"];
            }

            if (dct.ContainsKey("tool_paths"))
            {
                toolPaths = (List<string>)dct["tool_paths"];
            }

            if (dct.ContainsKey("component_paths"))
            {
                componentPaths = (List<string>)dct["component_paths"];
            }

            base.Set(dct);
        }

        /*
         * Cache Methods
         */

        public override void RemoveFromCache()
        {
            foreach (KeyValuePair<string, List<TraceDataPoint>> entry in data)
            {
                foreach (TraceDataPoint dp in entry.Value)
                {
                    dp.RemoveFromCache();
                }
            }
            base.RemoveFromCache();
        }

        public override void AddToCache()
        {
            foreach (KeyValuePair<string, List<TraceDataPoint>> entry in data)
            {
                foreach (TraceDataPoint dp in entry.Value)
                {
                    dp.AddToCache();
                }
            }
            base.AddToCache();
        }


        /*
         * Children Methods
         */

        public override bool DeleteChild(string uuid)
        {
            string group = null;
            foreach (KeyValuePair<string, List<TraceDataPoint>> entry in data)
            {
                foreach (TraceDataPoint dp in entry.Value)
                {
                    if (dp.uuid == uuid)
                    {
                        group = entry.Key;
                        break;
                    }
                }

                if (group != null)
                {
                    break;
                }
            }

            bool success = false;
            if (group != null)
            {
                DeleteDataPoint(group, uuid);
                success = true;
            }

            return success;
        }

        /*
         * Update Methods
         */

        public override void DeepUpdate()
        {
            foreach (KeyValuePair<string, List<TraceDataPoint>> entry in _data)
            {
                foreach (var dp in entry.Value)
                {
                    dp.DeepUpdate();
                }
            }

            base.DeepUpdate();

            UpdatedAttribute("data", "update");
            UpdatedAttribute("time", "update");
            UpdatedAttribute("end_effector_path", "update");
            UpdatedAttribute("joint_paths", "update");
            UpdatedAttribute("tool_paths", "update");
            UpdatedAttribute("component_paths", "update");
        }

        public override void ShallowUpdate()
        {
            base.ShallowUpdate();

            UpdatedAttribute("data", "update");
            UpdatedAttribute("time", "update");
            UpdatedAttribute("end_effector_path", "update");
            UpdatedAttribute("joint_paths", "update");
            UpdatedAttribute("tool_paths", "update");
            UpdatedAttribute("component_paths", "update");
        }
    }
}
