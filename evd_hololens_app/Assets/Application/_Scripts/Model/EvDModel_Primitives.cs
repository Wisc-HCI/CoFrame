using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace EvD
{
    [System.Serializable]
    public class Primitive : Node
    {

        /*
         * Constructors
         */

        public Primitive(string type = "", string name = "", string uuid = null,
                       Node parent = null, bool appendType = true)
        : base(appendType ? "primitive." + type : type, name, uuid, parent, appendType) { }

        public new static Primitive FromDict(Dictionary<string, object> dct)
        {
            return new Primitive(
                type: (string)dct["type"],
                name: (string)dct["name"],
                uuid: (string)dct["uuid"],
                appendType: false
            );
        }
    }

    [System.Serializable]
    public class MoveTrajectory  : Primitive
    {

        /*
         * Private Members
         */

        private string _startLocationUuid = null;
        private string _endLocationUuid = null;
        private List<Trajectory> _trajectories = null;
        private string _runnableTrajectoryUuid = null;

        /*
         * Constructors
         */

        public MoveTrajectory(string startLocUuid, string endLocUuid, List<Trajectory> trajectories = null, 
                              string runnableTrajUuid=null, string type = "", string name = "", 
                              string uuid = null, Node parent = null, bool appendType = true,
                              bool createDefault = true) 
        : base(appendType ? "move-trajectory." + type : type, name, uuid, parent, appendType)
        {
            this.startLocationUuid = startLocUuid;
            this.endLocationUuid = endLocUuid;
            this.trajectories = (trajectories == null) ? new List<Trajectory>() : trajectories;
            this.runnableTrajectoryUuid = runnableTrajUuid;

            if (this.trajectories.Count == 0 && createDefault)
            {
                var traj = new Trajectory(startLocUuid, endLocUuid, parent: this);
                AddTrajectory(traj);
            }
        }

        public new static MoveTrajectory FromDict(Dictionary<string,object> dct)
        {
            var trajectories = new List<Trajectory>();
            foreach (var t in (List<Dictionary<string,object>>)dct["trajectories"])
            {
                trajectories.Add(Trajectory.FromDict(t));
            }

            return new MoveTrajectory(
                startLocUuid: (string)dct["start_location_uuid"],
                endLocUuid: (string)dct["end_location_uuid"],
                trajectories: trajectories,
                runnableTrajUuid: (string)dct["runnable_trajectory_uuid"],
                type: (string)dct["type"],
                name: (string)dct["name"],
                uuid: (string)dct["uuid"],
                appendType: false
            );
        }

        public override Dictionary<string,object> ToDict()
        {
            var trajDct = new List<Dictionary<string, object>>();
            foreach (var t in trajectories)
            {
                trajDct.Add(t.ToDict());
            }

            var dct = base.ToDict();
            dct.Add("start_location_uuid", startLocationUuid);
            dct.Add("end_location_uuid", endLocationUuid);
            dct.Add("trajectories", trajDct);
            dct.Add("runnable_trajectory_uuid", runnableTrajectoryUuid);
            return dct;
        }

        /*
         * Accessors and Modifiers
         */

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
                    foreach (var traj in trajectories)
                    {
                        traj.startLocationUuid = value;
                    }
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
                    foreach (var traj in trajectories)
                    {
                        traj.endLocationUuid = value;
                    }
                    UpdatedAttribute("end_location_uuid", "set");
                }
            }
        }

        public string runnableTrajectoryUuid
        {
            get
            {
                return _runnableTrajectoryUuid;
            }

            set
            {
                if (_runnableTrajectoryUuid != value)
                {
                    if (SearchTrajectoriesForIndex(value) == -1 && value != null)
                    {
                        throw new System.Exception("Invalid trajectory selected");
                    }

                    // Must always have a runnable uuid set if there is at least one trajectory available
                    if (value == null && trajectories.Count > 0)
                    {
                        _runnableTrajectoryUuid = trajectories[0].uuid;
                    }
                    else
                    {
                        _runnableTrajectoryUuid = value;
                    }
                    
                    UpdatedAttribute("runnable_trajectory_uuid", "set");
                }
            }
        }

        public List<Trajectory> trajectories
        {
            get
            {
                return _trajectories;
            }

            set
            {
                if (_trajectories != value)
                {
                    foreach (var t in _trajectories)
                    {
                        t.RemoveFromCache();
                    }

                    _trajectories = value;
                    bool runnableFound = false;
                    foreach (var t in _trajectories)
                    {
                        t.parent = this;
                        if (t.uuid == runnableTrajectoryUuid)
                        {
                            runnableFound = true;
                        }
                    }

                    if (!runnableFound)
                    {
                        // must assign a runnable id if more than zero trajectories
                        if (_trajectories.Count > 0)
                        {
                            runnableTrajectoryUuid = _trajectories[0].uuid;
                        }
                        else
                        {
                            runnableTrajectoryUuid = null;
                        }
                    }
                    

                    UpdatedAttribute("trajectories", "set");
                }
            }
        }

        public void AddTrajectory(Trajectory traj)
        {
            traj.parent = this;
            trajectories.Add(traj);
            if (trajectories.Count == 1)
            {
                runnableTrajectoryUuid = traj.uuid;
            }
            UpdatedAttribute("trajectories", "add", traj.uuid);
        }

        public Trajectory GetTrajectory(string trjId)
        {
            return trajectories[SearchTrajectoriesForIndex(trjId)];
        }

        public void ReorderTrajectories(string uuid, int shift)
        {
            int idx = SearchTrajectoriesForIndex(uuid);

            if (idx != -1)
            {
                int shiftedIdx = idx + shift;
                if (shiftedIdx < 0 || shiftedIdx >= trajectories.Count)
                {
                    throw new System.Exception("Index out of bounds");
                }

                var copy = trajectories[idx];
                trajectories.RemoveAt(idx);
                trajectories.Insert(shiftedIdx, copy);
                UpdatedAttribute("trajectories", "reorder");
            }
        }

        public void DeleteTrajectory(string trjId)
        {
            int idx = SearchTrajectoriesForIndex(trjId);

            if (idx == -1)
            {
                throw new System.Exception("Trajectory not in list");
            }

            trajectories[idx].RemoveFromCache();
            trajectories.RemoveAt(idx);

            if (trjId == runnableTrajectoryUuid)
            {
                // must always have a runnable trajectory if trajectories length > 0
                if (trajectories.Count > 0)
                {
                    runnableTrajectoryUuid = trajectories[0].uuid;
                } 
                else
                {
                    runnableTrajectoryUuid = null;
                }
            }

            UpdatedAttribute("trajectories", "delete", trjId);
        }

        public int SearchTrajectoriesForIndex(string trjId)
        {
            int idx = -1;
            for (int i=0; i<trajectories.Count; i++)
            {
                if (trajectories[i].uuid == trjId)
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

            if (dct.ContainsKey("trajectories"))
            {
                var trajs = new List<Trajectory>();
                foreach (var t in (List<Dictionary<string, object>>)dct["trajectories"])
                {
                    trajs.Add(Trajectory.FromDict(t));
                }
                trajectories = trajs;
            }

            if (dct.ContainsKey("runnable_trajectory_uuid"))
            {
                runnableTrajectoryUuid = (string)dct["runnable_trajectory_uuid"];
            }

            base.Set(dct);
        }

        /*
         * Cache Methods
         */

        public override void RemoveFromCache()
        {
            foreach (var t in trajectories)
            {
                t.RemoveFromCache();
            }
            base.RemoveFromCache();
        }

        public override void AddToCache()
        {
            foreach (var t in trajectories)
            {
                t.AddToCache();
            }
            base.AddToCache();
        }

        /*
         * Children Methods
         */

        public override bool DeleteChild(string uuid)
        {
            bool success = false;

            foreach (var t in trajectories)
            {
                if (t.uuid == uuid)
                {
                    DeleteTrajectory(uuid);
                    success = true;
                    break;
                }
            }

            return success;
        }

        /*
         * Update Methods
         */

        public override void DeepUpdate()
        {
            foreach (var t in trajectories)
            {
                t.DeepUpdate();
            }

            base.DeepUpdate();

            UpdatedAttribute("start_location_uuid", "update");
            UpdatedAttribute("end_location_uuid", "update");
            UpdatedAttribute("trajectories", "update");
            UpdatedAttribute("runnable_trajectory_uuid", "update");
        }

        public override void ShallowUpdate()
        {
            base.ShallowUpdate();

            UpdatedAttribute("start_location_uuid", "update");
            UpdatedAttribute("end_location_uuid", "update");
            UpdatedAttribute("trajectories", "update");
            UpdatedAttribute("runnable_trajectory_uuid", "update");
        }
    }

    [System.Serializable]
    public class MoveUnplanned : Primitive
    {
        /*
         * Private Members
         */

        private string _locationUuid;
        private bool _manualSafety;

        /*
        * Constructors
        */

        public MoveUnplanned(string locationUuid, bool manualSafety, string type = "", string name = "",
                             string uuid = null, Node parent = null, bool appendType = true) 
        : base(appendType ? "move-unplanned." + type : type, name, uuid, parent, appendType)
        {
            this.locationUuid = locationUuid;
            this.manualSafety = manualSafety;
        }

        public new static MoveUnplanned FromDict(Dictionary<string,object> dct)
        {
            return new MoveUnplanned(
                locationUuid: (string)dct["location_uuid"],
                manualSafety: (bool)dct["manual_safety"],
                type: (string)dct["type"],
                name: (string)dct["name"],
                uuid: (string)dct["uuid"],
                appendType: false
            );
        }

        public override Dictionary<string,object> ToDict()
        {
            var dct = base.ToDict();
            dct.Add("location_uuid", locationUuid);
            dct.Add("manual_safety", manualSafety);
            return dct;
        }

        /*
         * Accessors and Modifiers
         */

        public string locationUuid
        {
            get
            {
                return _locationUuid;
            }

            set
            {
                if (_locationUuid != value)
                {
                    _locationUuid = value;
                    UpdatedAttribute("location_uuid", "set");
                }
            }
        }

        public bool manualSafety
        {
            get
            {
                return _manualSafety;
            }

            set
            {
                if (_manualSafety != value)
                {
                    _manualSafety = value;
                    UpdatedAttribute("manual_safety", "set");
                }
            }
        }

        public override void Set(Dictionary<string,object> dct)
        {
            if (dct.ContainsKey("location_uuid"))
            {
                locationUuid = (string)dct["location_uuid"];
            }

            if (dct.ContainsKey("manual_safety"))
            {
                manualSafety = (bool)dct["manual_safety"];
            }

            base.Set(dct);
        }

        /*
         * Update Methods
         */

        public override void DeepUpdate()
        {
            base.DeepUpdate();

            UpdatedAttribute("location_uuid", "update");
            UpdatedAttribute("manual_safety", "update");
        }

        public override void ShallowUpdate()
        {
            base.ShallowUpdate();

            UpdatedAttribute("location_uuid", "update");
            UpdatedAttribute("manual_safety", "update");
        }
    }

    [System.Serializable]
    public class Delay : Primitive
    {
        /*
         * Private Members
         */

        private float _duration;

        /*
         * Constructors
         */

        public Delay(float duration = 0, string type = "", string name = "", 
                     string uuid = null, Node parent = null, bool appendType = true) 
        : base(appendType ? "delay." + type : type, name, uuid, parent, appendType)
        {
            this.duration = duration;
        }

        public new static Delay FromDict(Dictionary<string,object> dct)
        {
            return new Delay(
                duration: (float)dct["duration"],
                type: (string)dct["type"],
                name: (string)dct["name"],
                uuid: (string)dct["uuid"],
                appendType: false
            );
        }

        public override Dictionary<string,object> ToDict()
        {
            var dct = base.ToDict();
            dct.Add("duration", duration);
            return dct;
        }

        /*
         * Accessors and Modifiers
         */

        public float duration
        {
            get
            {
                return _duration;
            }

            set
            {
                if (_duration != value)
                {
                    _duration = value;
                    UpdatedAttribute("duration", "set");
                }
            }
        }

        public override void Set(Dictionary<string,object> dct)
        {
            if (dct.ContainsKey("duration"))
            {
                duration = (float)dct["duration"];
            }

            base.Set(dct);
        }

        /*
         * Update Methods
         */

        public override void DeepUpdate()
        {
            base.DeepUpdate();

            UpdatedAttribute("duration", "update");
        }

        public override void ShallowUpdate()
        {
            base.ShallowUpdate();

            UpdatedAttribute("duration", "update");
        }
    }

    [System.Serializable]
    public class Gripper : Primitive
    {
        /*
         * Private Members
         */

        private float _position;
        private float _effort;
        private float _speed;

        /*
         * Constructors
         */

        public Gripper(float position = 0, float effort = 0, float speed = 0, string type = "", 
                       string name = "", string uuid = null, Node parent = null, bool appendType = true) 
        : base(appendType ? "gripper." + type : type, name, uuid, parent, appendType)
        {
            this.position = position;
            this.effort = effort;
            this.speed = speed;
        }

        public new static Gripper FromDict(Dictionary<string,object> dct)
        {
            return new Gripper(
                position: (float)dct["position"],
                effort: (float)dct["effort"],
                speed: (float)dct["speed"],
                type: (string)dct["type"],
                name: (string)dct["name"],
                uuid: (string)dct["uuid"],
                appendType: false
            );
        }

        public override Dictionary<string,object> ToDict()
        {
            var dct = base.ToDict();
            dct.Add("position", position);
            dct.Add("effort", effort);
            dct.Add("speed", speed);
            return dct;
        }

        /*
         * Accessors and Modifiers
         */

        public float position
        {
            get
            {
                return _position;
            }

            set
            {
                if (_position != value)
                {
                    _position = value;
                    UpdatedAttribute("position", "set");
                }
            }
        }

        public float effort
        {
            get
            {
                return _effort;
            }

            set
            {
                if (_effort != value)
                {
                    _effort = value;
                    UpdatedAttribute("effort", "set");
                }
            }
        }

        public float speed
        {
            get
            {
                return _speed;
            }

            set
            {
                if (_speed != value)
                {
                    _speed = value;
                    UpdatedAttribute("speed", "set");
                }
            }
        }

        public override void Set(Dictionary<string,object> dct)
        {

            if (dct.ContainsKey("position"))
            {
                position = (float)dct["position"];
            }

            if (dct.ContainsKey("effort"))
            {
                effort = (float)dct["effort"];
            }

            if (dct.ContainsKey("speed"))
            {
                speed = (float)dct["speed"];
            }

            base.Set(dct);
        }

        /*
         * Update Methods
         */

        public override void DeepUpdate()
        {
            base.DeepUpdate();

            UpdatedAttribute("position", "update");
            UpdatedAttribute("effort", "update");
            UpdatedAttribute("speed", "update");
        }

        public override void ShallowUpdate()
        {
            base.ShallowUpdate();

            UpdatedAttribute("position", "update");
            UpdatedAttribute("effort", "update");
            UpdatedAttribute("speed", "update");
        }
    }

    [System.Serializable]
    public class MachinePrimitive : Primitive
    {
        /*
         * Private Members
         */

        private string _machineUuid = null;

        /*
         * Constructors
         */

        public MachinePrimitive(string machineUuid, string type = "", string name = "", 
                                string uuid = null, Node parent = null, bool appendType = true)
        : base(appendType ? "machine-primitive." + type : type, name, uuid, parent, appendType)
        {
            this.machineUuid = machineUuid;
        }

        public new static MachinePrimitive FromDict(Dictionary<string, object> dct)
        {
            return new MachinePrimitive(
                machineUuid: (string)dct["machine_uuid"],
                type: (string)dct["type"],
                name: (string)dct["name"],
                uuid: (string)dct["uuid"],
                appendType: false
            );
        }

        public override Dictionary<string,object> ToDict()
        {
            var dct = base.ToDict();
            dct.Add("machine_uuid", machineUuid);
            return dct;
        }

        /*
         * Accessors and Modifiers
         */

        public string machineUuid
        {
            get
            {
                return _machineUuid;
            }

            set
            {
                if (_machineUuid != value)
                {
                    _machineUuid = value;
                    UpdatedAttribute("machine_uuid", "set");
                }
            }
        }

        public override void Set(Dictionary<string,object> dct)
        {
            if (dct.ContainsKey("machine_uuid"))
            {
                machineUuid = (string)dct["machine_uuid"];
            }

            base.Set(dct);
        }

        /*
         * Update Methods
         */

        public override void DeepUpdate()
        {
            base.DeepUpdate();

            UpdatedAttribute("machine_uuid", "update");
        }

        public override void ShallowUpdate()
        {
            base.ShallowUpdate();

            UpdatedAttribute("machine_uuid", "update");
        }
    }

    [System.Serializable]
    public class MachineStart : MachinePrimitive
    {

        /*
         * Constructors
         */

        public MachineStart(string machineUuid, string type = "", string name = "", string uuid = null,
                       Node parent = null, bool appendType = true)
        : base(machineUuid, appendType ? "machine-start." + type : type, name, uuid, parent, appendType) { }

        public new static MachineStart FromDict(Dictionary<string, object> dct)
        {
            return new MachineStart(
                machineUuid: (string)dct["machine_uuid"],
                type: (string)dct["type"],
                name: (string)dct["name"],
                uuid: (string)dct["uuid"],
                appendType: false
            );
        }
    }

    [System.Serializable]
    public class MachineWait : MachinePrimitive
    {

        /*
         * Constructors
         */

        public MachineWait(string machineUuid, string type = "", string name = "", string uuid = null,
                       Node parent = null, bool appendType = true)
        : base(machineUuid, appendType ? "machine-wait." + type : type, name, uuid, parent, appendType) { }

        public new static MachineWait FromDict(Dictionary<string, object> dct)
        {
            return new MachineWait(
                machineUuid: (string)dct["machine_uuid"],
                type: (string)dct["type"],
                name: (string)dct["name"],
                uuid: (string)dct["uuid"],
                appendType: false
            );
        }
    }

    [System.Serializable]
    public class MachineStop : MachinePrimitive
    {

        /*
         * Constructors
         */

        public MachineStop(string machineUuid, string type = "", string name = "", string uuid = null,
                       Node parent = null, bool appendType = true)
        : base(machineUuid, appendType ? "machine-stop." + type : type, name, uuid, parent, appendType) { }

        public new static MachineStop FromDict(Dictionary<string, object> dct)
        {
            return new MachineStop(
                machineUuid: (string)dct["machine_uuid"],
                type: (string)dct["type"],
                name: (string)dct["name"],
                uuid: (string)dct["uuid"],
                appendType: false
            );
        }
    }

    [System.Serializable]
    public class MachineInitialize : MachinePrimitive
    {

        /*
         * Constructors
         */

        public MachineInitialize(string machineUuid, string type = "", string name = "", string uuid = null,
                       Node parent = null, bool appendType = true)
        : base(machineUuid, appendType ? "machine-initialize." + type : type, name, uuid, parent, appendType) { }

        public new static MachineInitialize FromDict(Dictionary<string, object> dct)
        {
            return new MachineInitialize(
                machineUuid: (string)dct["machine_uuid"],
                type: (string)dct["type"],
                name: (string)dct["name"],
                uuid: (string)dct["uuid"],
                appendType: false
            );
        }
    }

    [System.Serializable]
    public class Breakpoint : Primitive
    {

        /*
         * Constructors
         */

        public Breakpoint(string type = "", string name = "", string uuid = null,
                          Node parent = null, bool appendType = true)
        : base(appendType ? "breakpoint." + type : type, name, uuid, parent, appendType) { }

        public new static Breakpoint FromDict(Dictionary<string, object> dct)
        {
            return new Breakpoint(
                type: (string)dct["type"],
                name: (string)dct["name"],
                uuid: (string)dct["uuid"],
                appendType: false
            );
        }
    }
}
