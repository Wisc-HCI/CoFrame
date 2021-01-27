using System.Collections;
using System.Collections.Generic;


namespace EvD
{
    namespace Data
    {

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
            private List<string> _waypoint_uuids = new List<string>();

            /*
            * Constructors
            */

            public Trajectory(string startLocUuid = null, string endLocUuid = null, List<string> waypoint_uuids = null, 
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
                    this.waypoint_uuids = waypoint_uuids;
                }
                
                this.trace = trace;
            }

            public static new Trajectory FromDict(Dictionary<string,object> dct)
            {
                var waypoint_uuids = new List<string>();
                foreach (var wp in dct["waypoint_uuids"])
                {
                    waypoint_uuids.Add((string)wp);
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
                    waypoint_uuids: waypoint_uuids,
                    trace: trace,
                    type: (string)dct["type"],
                    name: (string)dct["name"],
                    uuid: (string)dct["uuid"],
                    appendType: false
                );
            }

            public override Dictionary<string,object> ToDict()
            {
                var dctWaypoints = new List<string>();
                foreach (var wp in waypoint_uuids)
                {
                    dctWaypoints.Add(wp);
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
                dct.Add("waypoint_uuids", dctWaypoints);
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

            public List<string> waypoint_uuids
            {
                get
                {
                    return _waypoint_uuids;
                }

                set
                {
                    if (_waypoint_uuids != value)
                    {
                        foreach (var w in _waypoint_uuids)
                        {
                            w.RemoveFromCache();
                        }

                        _waypoint_uuids = value;
                        foreach (var w in _waypoint_uuids)
                        {
                            w.parent = this;
                        }

                        trace = null;
                        UpdatedAttribute("waypoint_uuids", "set");
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

            public void AddWaypointUuid(string uuid)
            {
                waypoint_uuids.append(uuid);
                trace = null;
                UpdatedAttribute("waypoint_uuids", "add", uuid);
            }

            public void InsertWaypointUuid(int idx, string uuid)
            {
                waypoint_uuids.Insert(idx, uuid);
                trace = null;
                UpdatedAttribute("waypoint_uuids", "add", uuid);
            }

            public void ReorderWaypoints(string uuid, int shift)
            {
                int idx = SearchWaypointsForIndex(uuid);

                if (idx != -1)
                {
                    int shiftedIdx = idx + shift;
                    if (shiftedIdx < 0 || shiftedIdx >= waypoint_uuids.Count)
                    {
                        throw new System.Exception("Index out of bounds");
                    }

                    var copy = waypoint_uuids[idx];
                    waypoint_uuids.RemoveAt(idx);
                    waypoint_uuids.Insert(shiftedIdx, copy);
                    UpdatedAttribute("waypoint_uuids", "reorder");
                }
            }

            public void DeleteWaypointUuid(string uuid)
            {
                int idx = SearchWaypointsForIndex(uuid);

                if (idx == -1)
                {
                    throw new System.Exception("Waypoint not in list");
                }

                waypoint_uuids.RemoveAt(idx);
                trace = null;
                UpdatedAttribute("waypoint_uuids", "delete", uuid);
            }

            public int SearchWaypointsForIndex(string uuid)
            {
                int idx = -1;
                for (int i = 0; i < waypoint_uuids.Count; i++)
                {
                    if (waypoint_uuids[i] == uuid)
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

                if (dct.ContainsKey("waypoint_uuids"))
                {
                    var waypoint_uuids = new List<string>();
                    foreach (var wp in dct["waypoint_uuids"])
                    {
                        waypoint_uuids.Add((string)wp);
                    }
                    this.waypoint_uuids = waypoint_uuids;
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
                if (trace != null)
                {
                    trace.RemoveFromCache();
                }

                base.RemoveFromCache();
            } 

            public override void AddToCache()
            {
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

                return success;
            }

            /*
            * Update Methods
            */

            public override void LateConstructUpdate()
            {
                if (trace != null)
                {
                    trace.LateConstructUpdate();
                }

                base.LateConstructUpdate();
            }

            public override void DeepUpdate()
            {
                if (trace != null)
                {
                    trace.DeepUpdate();
                }

                base.DeepUpdate();

                UpdatedAttribute("start_location_uuid", "update");
                UpdatedAttribute("end_location_uuid", "update");
                UpdatedAttribute("waypoint_uuids", "update");
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
                UpdatedAttribute("waypoint_uuids", "update");
                UpdatedAttribute("velocity", "update");
                UpdatedAttribute("acceleration", "update");
                UpdatedAttribute("move_type", "update");
                UpdatedAttribute("trace", "update");
            }
        }

    }
}