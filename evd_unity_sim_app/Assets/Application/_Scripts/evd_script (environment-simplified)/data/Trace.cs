using System.Collections;
using System.Collections.Generic;


namespace EvD
{
    namespace Data
    {

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

}