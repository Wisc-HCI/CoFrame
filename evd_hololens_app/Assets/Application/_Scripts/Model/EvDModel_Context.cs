using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.PlayerLoop;

namespace EvD
{
    [System.Serializable]
    public class Context : Node
    {
        /*
         * Private Members
         */

        private Context _parentContext = null;
        private Dictionary<string, Location> _locations = new Dictionary<string, Location>();
        private Dictionary<string, Machine> _machines = new Dictionary<string, Machine>();

        /*
         * Constructors
         */

        public Context(List<Location> locations = null, List<Machine> machines = null, 
                       Context parentContext = null, string type = "", string name = "", 
                       string uuid = null, Node parent = null, bool appendType = true)
        : base(appendType ? "context." + type : type, name, uuid, parent, appendType)
        {
            this.parentContext = parentContext;
            this.locations = locations;
            this.machines = machines;
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

            return new Context(
                locations: locations,
                machines: machines,
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

            var dct = base.ToDict();
            dct.Add("locations", locDct);
            dct.Add("machines", macDct);
            return dct;
        }

        /*
         * Accessors and Modifiers
         */

        public Context parentContext
        {
            get
            {
                return _parentContext;
            }

            set
            {
                if (_parentContext != value)
                {
                    _parentContext = parentContext;
                    UpdatedAttribute("parent_context", "set");
                }
            }
        }

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

            base.Set(dct);
        }

        /*
         * Cache Methods
         */

        public override void RemoveFromCache()
        {
            foreach (var entry in locations)
            {
                entry.RemoveFromCache();
            }

            foreach (var entry in machines)
            {
                entry.RemoveFromCache();
            }

            base.RemoveFromCache();
        }

        public override void AddToCache()
        {
            foreach (var entry in locations)
            {
                entry.AddToCache();
            }

            foreach (var entry in machines)
            {
                entry.AddToCache();
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

            return success;
        }

        /*
         * Update Methods
         */

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

            base.DeepUpdate();

            UpdatedAttribute("machines", "update");
            UpdatedAttribute("locations", "update");
        }

        public override void ShallowUpdate()
        {
            base.ShallowUpdate();

            UpdatedAttribute("machines", "update");
            UpdatedAttribute("locations", "update");
        }
    }
}
