using System.Collections;
using System.Collections.Generic;

using EvD.Data;


namespace EvD 
{
    [System.Serializable]
    public class Context : Node
    {
        /*
         * Private Members
         */

        private Dictionary<string, Location> _locations = new Dictionary<string, Location>();
        private Dictionary<string, Machine> _machines = new Dictionary<string, Machine>();
        private Dictionary<string, Thing> _things = new Dictionary<string, Thing>();
        private Dictionary<string, Waypoint> _waypoints = new Dictionary<string, Waypoint>();
        private Dictionary<string, Trajectory> _trajectories = new Dictionary<string, Trajectory>();

        /*
         * Constructors
         */

        public static new string TypeString()
        {
            return "context.";
        }

        public static new string FullTypeString()
        {
            return Node.FullTypeString() + TypeString();
        }
        
        public Context(List<Location> locations = null, List<Machine> machines = null, List<Thing> things = null, 
                       List<Waypoint> waypoints = null, List<Trajectory> trajectories = null, string type = "", 
                       string name = "", string uuid = null, Node parent = null, bool appendType = true) 
        : base(appendType ? "context." + type : type, name, uuid, parent, appendType)
        {
            this.locations = (locations != null) ? locations : new List<Location>();
            this.machines = (machines != null) ? machines : new List<Machine>();
            this.things = (things != null) ? things : new List<Thing>();
            this.waypoints = (waypoints != null) ? waypoints : new List<Waypoint>();
            this.trajectories = (trajectories != null) ? trajectories : new List<Trajectory>();
        }

        public new static Context FromDict(Dictionary<string,object> dct) 
        {
            var locations = new List<Location>();
            foreach(var locDct in (List<Dictionary<string,object>>)dct["locations"]) {
                locations.Add(Location.FromDict(locDct));
            }

            var machines = new List<Machine>();
            foreach (var macDct in (List<Dictionary<string, object>>)dct["machines"])
            {
                machines.Add(Machine.FromDict(macDct));
            }

            var things = new List<Thing>();
            foreach (var thDct in (List<Dictionary<string, object>>)dct["things"])
            {
                things.Add(Thing.FromDict(thDct));
            }

            var waypoints = new List<Waypoint>();
            foreach (var wpDct in (List<Dictionary<string, object>>)dct["waypoints"])
            {
                waypoints.Add(Waypoint.FromDict(wpDct));
            }

            var trajectories = new List<Trajectory>();
            foreach (var traDct in (List<Dictionary<string, object>>)dct["trajectories"])
            {
                trajectories.Add(Trajectory.FromDict(traDct));
            }

            return new Context(
                locations: locations,
                machines: machines,
                things: things,
                waypoints: waypoints,
                trajectories: trajectories,
                type: (string)dct["type"],
                name: (string)dct["name"],
                uuid: (string)dct["uuid"],
                appendType: false
            );
        }

        public override Dictionary<string,object> ToDict()
        {
            var locDct = new List<Dictionary<string, object>>();
            foreach (var entry in locations)
            {
                locDct.Add(entry.ToDict());
            }

            var macDct = new List<Dictionary<string, object>>();
            foreach (var entry in machines)
            {
                macDct.Add(entry.ToDict());
            }

            var thDct = new List<Dictionary<string, object>>();
            foreach (var entry in things) 
            {
                thDct.Add(entry.ToDict());
            }

            var wpDct = new List<Dictionary<string, object>>();
            foreach (var entry in waypoints) 
            {
                wpDct.Add(entry.ToDict());
            }

            var traDct = new List<Dictionary<string, object>>();
            foreach (var entry in trajectories) 
            {
                traDct.Add(entry.ToDict());
            }

            var dct = base.ToDict();
            dct.Add("locations", locDct);
            dct.Add("machines", macDct);
            dct.Add("things",thDct);
            dct.Add("waypoints",wpDct);
            dct.Add("trajectories",traDct);
            return dct;
        }

        /*
         * Accessors and Modifiers
         */

        public List<Location> locations
        {
            get
            {
                return new List<Location>(_locations.Values);
            }

            set
            {
                if (_locations != null)
                {
                    foreach (KeyValuePair<string, Location> entry in _locations)
                    {
                        entry.Value.RemoveFromCache();
                    }
                }

                _locations.Clear();
                if (value != null)
                {
                    foreach (var l in value)
                    {
                        l.parent = this;
                        _locations.Add(l.uuid, l);
                    }
                }

                UpdatedAttribute("locations", "set");
            }
        }

        public void AddLocation(Location l)
        {
            l.parent = this;
            _locations.Add(l.uuid, l);
            UpdatedAttribute("locations", "add", l.uuid);
        }

        public void DeleteLocation(string uuid)
        {
            _locations[uuid].RemoveFromCache();
            _locations.Remove(uuid);
            UpdatedAttribute("locations", "delete", uuid);
        }

        public Location GetLocation(string uuid)
        {
            return _locations[uuid]; 
        }

        public List<Machine> machines
        {
            get
            {
                return new List<Machine>(_machines.Values);
            }

            set
            {
                if (_machines != null)
                {
                    foreach (KeyValuePair<string, Machine> entry in _machines)
                    {
                        entry.Value.RemoveFromCache();
                    }
                }

                _machines.Clear();
                if (value != null)
                {
                    foreach (var m in value)
                    {
                        m.parent = this;
                        _machines.Add(m.uuid, m);
                    }
                }
                UpdatedAttribute("machines", "set");
            }
        }

        public void AddMachine(Machine m)
        {
            m.parent = this;
            _machines.Add(m.uuid, m);
            UpdatedAttribute("machines", "add", m.uuid);
        }

        public void DeleteMachine(string uuid)
        {
            _machines[uuid].RemoveFromCache();
            _machines.Remove(uuid);
            UpdatedAttribute("machines", "delete", uuid);
        }

        public Machine GetMachine(string uuid)
        {
            return _machines[uuid];
        }

        public List<Thing> things
        {
            get
            {
                return new List<Thing>(_things.Values);
            }

            set
            {
                if (_things != null)
                {
                    foreach (KeyValuePair<string, Thing> entry in _things)
                    {
                        entry.Value.RemoveFromCache();
                    }
                }

                _things.Clear();
                if (value != null)
                {
                    foreach (var t in value)
                    {
                        t.parent = this;
                        _things.Add(t.uuid, t);
                    }
                }
                UpdatedAttribute("things", "set");
            }
        }

        public void AddThing(Thing t)
        {
            t.parent = this;
            _things.Add(t.uuid, t);
            UpdatedAttribute("things", "add", t.uuid);
        }

        public void DeleteThing(string uuid)
        {
            _things[uuid].RemoveFromCache();
            _things.Remove(uuid);
            UpdatedAttribute("things", "delete", uuid);
        }

        public Thing GetThing(string uuid)
        {
            return _things[uuid];
        }

        public List<Waypoint> waypoints 
        {
            get 
            {
                return new List<Waypoint>(_waypoints.Values);
            }

            set
            {
                if (_waypoints != null) 
                {
                    foreach (KeyValuePair<string, Waypoint> entry in _waypoints)
                    {
                        entry.Value.RemoveFromCache();
                    }
                }

                _waypoints.Clear();
                if (value != null)
                {
                    foreach (var w in value)
                    {
                        w.parent = this;
                        _waypoints.Add(w.uuid, w);
                    }
                }
                UpdatedAttribute("waypoints", "set");
            }
        }

        public void AddWaypoint(Waypoint w) 
        {
            w.parent = this;
            _waypoints.Add(w.uuid, w);
            UpdatedAttribute("waypoints", "add", w.uuid);
        }

        public void DeleteWaypoint(string uuid)
        {
            _waypoints[uuid].RemoveFromCache();
            _waypoints.Remove(uuid);
            UpdatedAttribute("waypoints", "delete", uuid);
        }

        public Waypoint GetWaypoint(string uuid)
        {
            return _waypoints[uuid];
        }
    
        public List<Trajectory> trajectories
        {
            get 
            {
                return new List<Trajectory>(_trajectories.Values);
            }

            set 
            {
                if (_trajectories != null) 
                {
                    foreach (KeyValuePair<string, Trajectory> entry in _trajectories)
                    {
                        entry.Value.RemoveFromCache();
                    }
                }

                _trajectories.Clear();
                if (value != null) 
                {
                    foreach (var t in value)
                    {
                        t.parent = this;
                        _trajectories.Add(t.uuid, t);
                    }
                }
                UpdatedAttribute("trajectories","set");
            }
        }

        public void AddTrajectory(Trajectory t) 
        {
            t.parent = this;
            _trajectories.Add(t.uuid, t);
            UpdatedAttribute("trajectories", "add", t.uuid);
        }

        public void DeleteTrajectory(string uuid)
        {
            _trajectories[uuid].RemoveFromCache();
            _trajectories.Remove(uuid);
            UpdatedAttribute("trajectories","delete",uuid);
        }

        public Trajectory GetTrajectory(string uuid)
        {
            return _trajectories[uuid];
        }
    
        public override void Set(Dictionary<string, object> dct)
        {
            if (dct.ContainsKey("machines"))
            {
                var machines = new List<Machine>();
                foreach (var macDct in (List<Dictionary<string, object>>)dct["machines"])
                {
                    machines.Add(Machine.FromDict(macDct));
                }

                this.machines = machines;
            }

            if (dct.ContainsKey("locations"))
            {
                var locations = new List<Location>();
                foreach (var locDct in (List<Dictionary<string, object>>)dct["locations"])
                {
                    locations.Add(Location.FromDict(locDct));
                }

                this.locations = locations;
            }

            if (dct.ContainsKey("things"))
            {
                var things = new List<Thing>();
                foreach (var thDct in (List<Dictionary<string, object>>)dct["things"])
                {
                    things.Add(Thing.FromDict(thDct));
                }

                this.things = things;
            }

            if (dct.ContainsKey("waypoints"))
            {
                var waypoints = new List<Waypoint>();
                foreach (var wpDct in (List<Dictionary<string, object>>)dct["waypoints"])
                {
                    waypoints.Add(Waypoint.FromDict(wpDct));
                }
                this.waypoints = waypoints;
            }

            if (dct.ContainsKey("trajectories"))
            {
                var trajectories = new List<Trajectory>();
                foreach (var traDct in (List<Dictionary<string, object>>)dct["trajectories"])
                {
                    trajectories.Add(Trajectory.FromDict(traDct));
                }
                this.trajectories = trajectories;
            }

            base.Set(dct);
        }
    
        /*
         * Cache Methods
         */

        public override void RemoveFromCache()
        {
            foreach (var m in machines)
            {
                m.RemoveFromCache();
            }

            foreach (var l in locations)
            {
                l.RemoveFromCache();
            }

            foreach (var t in things)
            {
                t.RemoveFromCache();
            }

            foreach (var w in waypoints)
            {
                w.RemoveFromCache();
            }

            foreach (var t in trajectories)
            {
                t.RemoveFromCache();
            }

            base.RemoveFromCache();
        }

        public override void AddToCache()
        {
            foreach (var m in machines)
            {
                m.AddToCache();
            }

            foreach (var l in locations)
            {
                l.AddToCache();
            }

            foreach (var t in things)
            {
                t.AddToCache();
            }

            foreach (var w in waypoints)
            {
                w.AddToCache();
            }

            foreach (var t in trajectories)
            {
                t.AddToCache();
            }

            base.AddToCache();
        }

        /*
         * Children Methods (Optional)
         */

        public override bool DeleteChild(string uuid)
        {
            bool success = false;

            if (_locations.ContainsKey(uuid))
            {
                DeleteLocation(uuid);
                success = true;
            }

            if (_machines.ContainsKey(uuid))
            {
                DeleteMachine(uuid);
                success = true;
            }

            if (_things.ContainsKey(uuid))
            {
                DeleteThing(uuid);
                success = true;
            }

            if (_waypoints.ContainsKey(uuid))
            {
                DeleteWaypoint(uuid);
                success = true;
            }

            if (_trajectories.ContainsKey(uuid)) 
            {
                DeleteTrajectory(uuid);
                success = true;
            }

            return success;
        }

        /*
         * Update Methods
         */

        public override void LateConstructUpdate()
        {
            foreach (var m in machines)
            {
                m.LateConstructUpdate();
            }

            foreach (var l in locations)
            {
                l.LateConstructUpdate();
            }

            foreach (var t in things)
            {
                t.LateConstructUpdate();
            }

            foreach (var w in waypoints)
            {
                w.LateConstructUpdate();
            }

            foreach (var t in trajectories)
            {
                t.LateConstructUpdate();
            }

            base.LateConstructUpdate();
        }

        public override void DeepUpdate()
        {
            foreach (var m in machines)
            {
                m.DeepUpdate();
            }

            foreach (var l in locations)
            {
                l.DeepUpdate();
            }

            foreach (var t in things)
            {
                t.DeepUpdate();
            }

            foreach (var w in waypoints)
            {
                w.DeepUpdate();
            }

            foreach (var t in trajectories)
            {
                t.DeepUpdate();
            }

            base.DeepUpdate();

            UpdatedAttribute("machines", "update");
            UpdatedAttribute("locations", "update");
            UpdatedAttribute("things", "update");
            UpdatedAttribute("waypoints", "update");
            UpdatedAttribute("trajectories", "update");
        }

        public override void ShallowUpdate()
        {
            base.ShallowUpdate();

            UpdatedAttribute("machines", "update");
            UpdatedAttribute("locations", "update");
            UpdatedAttribute("things", "update");
            UpdatedAttribute("waypoints", "update");
            UpdatedAttribute("trajectories", "update");
        }
    }
}