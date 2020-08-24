using RosSharp.RosBridgeClient.Messages;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.PlayerLoop;

namespace Cobots
{
    [System.Serializable]
    public class PinchPoint : Node
    {
        /*
         * Constants
         */

        public static readonly string GOOD_STATE = "good";
        public static readonly string WARN_STATE = "warn";
        public static readonly string ERROR_STATE = "error";

        /*
         * Private Members
         */

        private string _state = null;

        /*
         * Constructors
         */

        public PinchPoint(string state = null, string type = "", string name = "", string uuid = null, Node parent = null, bool appendType = true) 
        : base(appendType ? "pinch-point." + type : type, name, uuid, parent, appendType)
        {
            if (state == null)
            {
                state = GOOD_STATE;
            }
            this.state = state;
        }

        public new static PinchPoint FromDict(Dictionary<string,object> dct)
        {
            return new PinchPoint(
                state: (string)(dct["state"]),
                type: (string)(dct["type"]),
                name: (string)(dct["name"]),
                uuid: (string)(dct["uuid"])
            );
        }

        public override Dictionary<string,object> ToDict()
        {
            var dct = base.ToDict();
            dct.Add("state", state);
            return dct;
        }

        /*
         * Accessors and Modifiers
         */

        public string state
        {
            get
            {
                return _state;
            }

            set
            {
                if (value != GOOD_STATE && value != WARN_STATE && value != ERROR_STATE)
                {
                    throw new System.Exception("Invalid state provided");
                }

                if (_state != value)
                {
                    _state = value;
                    UpdatedAttribute("state","set");
                }
            }
        }

        public void SetStateGood()
        {
            _state = GOOD_STATE;
            UpdatedAttribute("state", "set");
        }

        public void SetStateWarn()
        {
            _state = WARN_STATE;
            UpdatedAttribute("state", "set");
        }

        public void SetStateError()
        {
            _state = ERROR_STATE;
            UpdatedAttribute("state", "set");
        }

        public override void Set(Dictionary<string, object> dct)
        {
            if (dct.ContainsKey("state"))
            {
                state = (string)dct["state"];
            }

            base.Set(dct);
        }

        /*
         * Update Methods
         */

        public override void DeepUpdate()
        {
            base.DeepUpdate();

            UpdatedAttribute("state", "update");
        }

        public override void ShallowUpdate()
        {
            base.ShallowUpdate();

            UpdatedAttribute("state", "update");
        }
    }

    [System.Serializable]
    public class CollisionMesh : Node
    {
        /*
         * Constants
         */

        public static readonly string GOOD_STATE = "good";
        public static readonly string WARN_STATE = "warn";
        public static readonly string ERROR_STATE = "error";

        /*
         * Private Members
         */

        private string _state = null;

        /*
         * Constructors
         */

        public CollisionMesh(string state = null, string type = "", string name = "", string uuid = null, Node parent = null, bool appendType = true)
        : base(appendType ? "collision-mesh." + type : type, name, uuid, parent, appendType)
        {
            if (state == null)
            {
                state = GOOD_STATE;
            }
            this.state = state;
        }

        public new static CollisionMesh FromDict(Dictionary<string, object> dct)
        {
            return new CollisionMesh(
                state: (string)(dct["state"]),
                type: (string)(dct["type"]),
                name: (string)(dct["name"]),
                uuid: (string)(dct["uuid"])
            );
        }

        public override Dictionary<string, object> ToDict()
        {
            var dct = base.ToDict();
            dct.Add("state", state);
            return dct;
        }

        /*
         * Accessors and Modifiers
         */

        public string state
        {
            get
            {
                return _state;
            }

            set
            {
                if (value != GOOD_STATE && value != WARN_STATE && value != ERROR_STATE)
                {
                    throw new System.Exception("Invalid state provided");
                }

                if (_state != value)
                {
                    _state = value;
                    UpdatedAttribute("state", "set");
                }
            }
        }

        public void SetStateGood()
        {
            _state = GOOD_STATE;
            UpdatedAttribute("state", "set");
        }

        public void SetStateWarn()
        {
            _state = WARN_STATE;
            UpdatedAttribute("state", "set");
        }

        public void SetStateError()
        {
            _state = ERROR_STATE;
            UpdatedAttribute("state", "set");
        }

        public override void Set(Dictionary<string, object> dct)
        {
            if (dct.ContainsKey("state"))
            {
                state = (string)dct["state"];
            }

            base.Set(dct);
        }

        /*
         * Update Methods
         */

        public override void DeepUpdate()
        {
            base.DeepUpdate();

            UpdatedAttribute("state", "update");
        }

        public override void ShallowUpdate()
        {
            base.ShallowUpdate();

            UpdatedAttribute("state", "update");
        }
    }

    [System.Serializable]
    public class ReachSphere : Node
    {
        /*
         * Constants
         */

        public static readonly string GOOD_STATE = "good";
        public static readonly string WARN_STATE = "warn";
        public static readonly string ERROR_STATE = "error";

        /*
         * Private Members
         */

        private string _state = null;

        /*
         * Constructors
         */

        public ReachSphere(string state = null, string type = "", string name = "", string uuid = null, Node parent = null, bool appendType = true)
        : base(appendType ? "reach-sphere." + type : type, name, uuid, parent, appendType)
        {
            if (state == null)
            {
                state = GOOD_STATE;
            }
            this.state = state;
        }

        public new static ReachSphere FromDict(Dictionary<string, object> dct)
        {
            return new ReachSphere(
                state: (string)(dct["state"]),
                type: (string)(dct["type"]),
                name: (string)(dct["name"]),
                uuid: (string)(dct["uuid"])
            );
        }

        public override Dictionary<string, object> ToDict()
        {
            var dct = base.ToDict();
            dct.Add("state", state);
            return dct;
        }

        /*
         * Accessors and Modifiers
         */

        public string state
        {
            get
            {
                return _state;
            }

            set
            {
                if (value != GOOD_STATE && value != WARN_STATE && value != ERROR_STATE)
                {
                    throw new System.Exception("Invalid state provided");
                }

                if (_state != value)
                {
                    _state = value;
                    UpdatedAttribute("state", "set");
                }
            }
        }

        public void SetStateGood()
        {
            _state = GOOD_STATE;
            UpdatedAttribute("state", "set");
        }

        public void SetStateWarn()
        {
            _state = WARN_STATE;
            UpdatedAttribute("state", "set");
        }

        public void SetStateError()
        {
            _state = ERROR_STATE;
            UpdatedAttribute("state", "set");
        }

        public override void Set(Dictionary<string, object> dct)
        {
            if (dct.ContainsKey("state"))
            {
                state = (string)dct["state"];
            }

            base.Set(dct);
        }

        /*
         * Update Methods
         */

        public override void DeepUpdate()
        {
            base.DeepUpdate();

            UpdatedAttribute("state", "update");
        }

        public override void ShallowUpdate()
        {
            base.ShallowUpdate();

            UpdatedAttribute("state", "update");
        }
    }

    [System.Serializable]
    public class OccupancyZone : Node
    {
        /*
         * Constants
         */

        public static readonly string HUMAN_TYPE = "human";
        public static readonly string ROBOT_TYPE = "robot";

        /*
         * Private Members
         */

        private float _positionX;
        private float _positionZ;
        private float _scaleX;
        private float _scaleZ;
        private string _occupancyType;

        /*
         * Constructors
         */

        public OccupancyZone(string occupancy_type, float posX = 0, float posZ = 0, float sclX = 1, float sclZ = 1, 
                             string type = "", string name = "", string uuid = null, Node parent = null, bool appendType = true)
        : base(appendType ? "occupancy-zone." + type : type, name, uuid, parent, appendType)
        {
            occupancyType = occupancy_type;

            positionX = posX;
            positionZ = posZ;
            scaleX = sclX;
            scaleZ = sclZ;
        }

        public new static OccupancyZone FromDict(Dictionary<string, object> dct)
        {
            return new OccupancyZone(
                occupancy_type: (string)(dct["occupancy_type"]),
                posX: (float)(dct["position_x"]),
                posZ: (float)(dct["position_z"]),
                sclX: (float)(dct["scale_x"]),
                sclZ: (float)(dct["scale_z"]),
                type: (string)(dct["type"]),
                name: (string)(dct["name"]),
                uuid: (string)(dct["uuid"])
            );
        }

        public override Dictionary<string, object> ToDict()
        {
            var dct = base.ToDict();
            dct.Add("occupancy_type", occupancyType);
            dct.Add("position_x", positionX);
            dct.Add("position_z", positionZ);
            dct.Add("scale_x", scaleX);
            dct.Add("scale_z", scaleZ);
            return dct;
        }

        /*
         * Accessors and Modifiers
         */

        public string occupancyType
        {
            get
            {
                return _occupancyType;
            }

            private set
            {
                if (_occupancyType != value)
                {
                    if (value != HUMAN_TYPE && value != ROBOT_TYPE)
                    {
                        throw new System.Exception("Invalid occupancy zone type specified");
                    }

                    _occupancyType = value;
                    UpdatedAttribute("occupancy_type", "set");
                }
            }
        }

        public float positionX
        {
            get
            {
                return _positionX;
            }

            set
            {
                if (_positionX != value)
                {
                    _positionX = value;
                    UpdatedAttribute("position_x", "set");
                }
            }
        }

        public float positionZ
        {
            get
            {
                return _positionZ;
            }

            set
            {
                if (_positionZ != value)
                {
                    _positionZ = value;
                    UpdatedAttribute("position_z", "set");
                }
            }
        }

        public float scaleX
        {
            get
            {
                return _scaleX;
            }

            set
            {
                if (_scaleX != value)
                {
                    _scaleX = value;
                    UpdatedAttribute("scale_x", "set");
                }
            }
        }

        public float scaleZ
        {
            get
            {
                return _scaleZ;
            }

            set
            {
                if (_scaleZ != value)
                {
                    _scaleZ = value;
                    UpdatedAttribute("scale_z", "set");
                }
            }
        }

        public override void Set(Dictionary<string, object> dct)
        {
            if (dct.ContainsKey("occupancy_type"))
            {
                occupancyType = (string)dct["occupancy_type"];
            }

            if (dct.ContainsKey("position_x"))
            {
                positionX = (float)dct["position_x"];
            }

            if (dct.ContainsKey("position_z"))
            {
                positionZ = (float)dct["position_z"];
            }

            if (dct.ContainsKey("scale_x"))
            {
                scaleX = (float)dct["scale_x"];
            }

            if (dct.ContainsKey("scale_z"))
            {
                scaleZ = (float)dct["scale_z"];
            }

            base.Set(dct);
        }

        /*
         * Update Methods
         */

        public override void DeepUpdate()
        {
            base.DeepUpdate();

            UpdatedAttribute("occupancy_type", "update");
            UpdatedAttribute("position_x", "update");
            UpdatedAttribute("position_z", "update");
            UpdatedAttribute("scale_x", "update");
            UpdatedAttribute("scale_z", "update");
        }

        public override void ShallowUpdate()
        {
            base.ShallowUpdate();

            UpdatedAttribute("occupancy_type", "update");
            UpdatedAttribute("position_x", "update");
            UpdatedAttribute("position_z", "update");
            UpdatedAttribute("scale_x", "update");
            UpdatedAttribute("scale_z", "update");
        }
    }

    [System.Serializable]
    public class Environment : Node
    {
        /*
         * Attributes
         */

        private Cache _cache = new Cache();
        public System.Action<List<Dictionary<string, string>>> changesCallback { get; set; } = null;

        private ReachSphere _reachSphere = null;
        private List<PinchPoint> _pinchPoints = null;
        private List<CollisionMesh> _collisionMeshes = null;
        private List<OccupancyZone> _occupancyZones = null;
        private List<Location> _locations = null;
        private List<Trajectory> _trajectories = null;

        /*
         * Constructors
         */

        public Environment(ReachSphere reachSphere, List<PinchPoint> pinchPoints, List<CollisionMesh> collisionMeshes,
                           List<OccupancyZone> occupancyZones, List<Location> locations, List<Trajectory> trajectories,
                           System.Action<List<Dictionary<string, string>>> changesCallback = null,
                           string type = "", string name = "", string uuid = null, Node parent = null, bool appendType = true)
        : base(appendType ? "environment." + type : type, name, uuid, parent, appendType)
        {
            this.changesCallback = changesCallback;
            this.reachSphere = reachSphere;
            this.pinchPoints = pinchPoints;
            this.collisionMeshes = collisionMeshes;
            this.occupancyZones = occupancyZones;
            this.locations = locations;
            this.trajectories = trajectories;
        }

        public static Environment FromDict(Dictionary<string, object> dct, System.Action<List<Dictionary<string, string>>> changesCallback = null)
        {
            var pinchPoints = new List<PinchPoint>();
            foreach (var p in (List<Dictionary<string, object>>)dct["pinch_points"])
            {
                pinchPoints.Add(PinchPoint.FromDict(p));
            }

            var collisionMeshes = new List<CollisionMesh>();
            foreach (var c in (List<Dictionary<string, object>>)dct["collision_meshes"])
            {
                collisionMeshes.Add(CollisionMesh.FromDict(c));
            }

            var occupancyZones = new List<OccupancyZone>();
            foreach (var o in (List<Dictionary<string,object>>)dct["occupancy_zones"])
            {
                occupancyZones.Add(OccupancyZone.FromDict(o));
            }

            var locations = new List<Location>();
            foreach (var l in (List<Dictionary<string, object>>)dct["locations"])
            {
                locations.Add(Location.FromDict(l));
            }

            var trajectories = new List<Trajectory>();
            foreach (var t in (List<Dictionary<string, object>>)dct["trajectories"])
            {
                trajectories.Add(Trajectory.FromDict(t));
            }

            return new Environment(
                reachSphere: ReachSphere.FromDict((Dictionary<string,object>)dct["reach_sphere"]),
                pinchPoints: pinchPoints,
                collisionMeshes: collisionMeshes,
                occupancyZones: occupancyZones,
                locations: locations,
                trajectories: trajectories,
                type: (string)dct["type"],
                name: (string)dct["name"],
                uuid: (string)dct["uuid"],
                appendType: false
            );
        }

        public override Dictionary<string, object> ToDict()
        {

            var pinchDct = new List<Dictionary<string, object>>();
            foreach (var p in pinchPoints)
            {
                pinchDct.Add(p.ToDict());
            }

            var collisionDct = new List<Dictionary<string, object>>();
            foreach (var c in collisionMeshes)
            {
                collisionDct.Add(c.ToDict());
            }

            var occupancyDct = new List<Dictionary<string, object>>();
            foreach (var o in occupancyZones)
            {
                occupancyDct.Add(o.ToDict());
            }

            var locationDct = new List<Dictionary<string, object>>();
            foreach (var l in locations)
            {
                locationDct.Add(l.ToDict());
            }

            var trajectoryDct = new List<Dictionary<string, object>>();
            foreach (var t in trajectories)
            {
                trajectoryDct.Add(t.ToDict());
            }

            var dct = base.ToDict();
            dct.Add("reach_sphere", reachSphere.ToDict());
            dct.Add("pinch_points", pinchDct);
            dct.Add("collision_meshes", collisionDct);
            dct.Add("occupancy_zones", occupancyDct);
            dct.Add("locations", locationDct);
            dct.Add("trajectories", trajectoryDct);
            return dct;
        }

        /*
         * Accessors and Modifiers
         */

        public override Cache cache
        {
            get
            {
                return _cache;
            }
        }

        public ReachSphere reachSphere
        {
            get
            {
                return _reachSphere;
            }

            set
            {
                if (value == null)
                {
                    throw new System.Exception("Reach sphere cannot be null");
                }

                if (_reachSphere != value)
                {
                    if (_reachSphere != null) // Initial State is null
                    {
                        _reachSphere.RemoveFromCache();
                    }
                    
                    _reachSphere = value;
                    _reachSphere.parent = this;
                    UpdatedAttribute("reach_sphere", "set", _reachSphere.uuid);
                }
            }
        }

        public List<PinchPoint> pinchPoints
        {
            get
            {
                return _pinchPoints;
            }

            set
            {
                if (value == null)
                {
                    throw new System.Exception("Pinch point list cannot be null");
                }

                if (_pinchPoints != value)
                {
                    if (_pinchPoints != null) // Initial State is null
                    {
                        foreach (var p in _pinchPoints)
                        {
                            p.RemoveFromCache();
                        }
                    }

                    _pinchPoints = value;

                    foreach (var p in _pinchPoints)
                    {
                        p.parent = this;
                    }

                    UpdatedAttribute("pinch_points", "set");
                }
            }
        }

        public List<CollisionMesh> collisionMeshes
        {
            get
            {
                return _collisionMeshes;
            }

            set
            {
                if (value == null)
                {
                    throw new System.Exception("Collision meshes list cannot be null");
                }

                if (_collisionMeshes != value)
                {
                    if (_collisionMeshes != null) // Initial State is null
                    {
                        foreach (var c in _collisionMeshes)
                        {
                            c.RemoveFromCache();
                        }
                    }
                    
                    _collisionMeshes = value;

                    foreach (var c in _collisionMeshes)
                    {
                        c.parent = this;
                    }

                    UpdatedAttribute("collision_meshes", "set");
                }
            }
        }

        public List<OccupancyZone> occupancyZones
        {
            get
            {
                return _occupancyZones;
            }
            
            set
            {
                if (value == null)
                {
                    throw new System.Exception("Occupancy zone list cannot be null");
                }

                if (_occupancyZones != value)
                {
                    if (_occupancyZones != null) // Initial State is null
                    {
                        foreach (var o in _occupancyZones)
                        {
                            o.RemoveFromCache();
                        }
                    }

                    _occupancyZones = value;

                    foreach (var o in _occupancyZones)
                    {
                        o.parent = this;
                    }

                    UpdatedAttribute("occupancy_zones", "set");
                }
            }
        }

        public List<Location> locations
        {
            get
            {
                return _locations;
            }

            set
            {
                if (value == null)
                {
                    throw new System.Exception("Location list cannot be null");
                }

                if (_locations != value)
                {
                    if (_locations != null) // Initial State is null
                    {
                        foreach (var l in _locations)
                        {
                            l.RemoveFromCache();
                        }
                    }

                    _locations = value;

                    foreach (var l in _locations)
                    {
                        l.parent = this;
                    }
                }
            }
        }

        public void AddLocation(Location l)
        {
            l.parent = this;
            _locations.Add(l);
            UpdatedAttribute("locations", "add", l.uuid);
        }

        public void DeleteLocation(string uuid)
        {
            int idx = _locations.FindIndex(x => x.uuid == uuid);

            _locations[idx].RemoveFromCache();
            _locations.RemoveAt(idx);
            UpdatedAttribute("locations", "delete", uuid);
        }

        public List<Trajectory> trajectories
        {
            get
            {
                return _trajectories;
            }

            set
            {
                if (value == null)
                {
                    throw new System.Exception("Trajectories list must not be null");
                }

                if (_trajectories != value)
                {
                    if (_trajectories != null) // initial state 
                    {
                        foreach (var t in _trajectories)
                        {
                            t.RemoveFromCache();
                        }
                    }

                    _trajectories = value;

                    foreach (var t in _trajectories)
                    {
                        t.parent = this;
                    }

                    UpdatedAttribute("trajectories", "set");
                }
            }
        }

        public void AddTrajectory(Trajectory trj)
        {
            trj.parent = this;
            _trajectories.Add(trj);
            UpdatedAttribute("trajectories", "add", trj.uuid);
        }

        public void DeleteTrajectory(string uuid)
        {
            int idx = _trajectories.FindIndex(x => x.uuid == uuid);

            _trajectories[idx].RemoveFromCache();
            _trajectories.RemoveAt(idx);
            UpdatedAttribute("trajectories", "delete", uuid);
        }

        public override void Set(Dictionary<string, object> dct)
        {
            if (dct.ContainsKey("reach_sphere"))
            {
                reachSphere = ReachSphere.FromDict((Dictionary<string, object>)dct["reach_sphere"]);
            }

            if (dct.ContainsKey("pinch_points"))
            {
                var pinchPoints = new List<PinchPoint>();
                foreach (var p in (List<Dictionary<string, object>>)dct["pinch_points"])
                {
                    pinchPoints.Add(PinchPoint.FromDict(p));
                }

                this.pinchPoints = pinchPoints;
            }

            if (dct.ContainsKey("collision_meshes"))
            {
                var collisionMeshes = new List<CollisionMesh>();
                foreach (var c in (List<Dictionary<string, object>>)dct["collision_meshes"])
                {
                    collisionMeshes.Add(CollisionMesh.FromDict(c));
                }

                this.collisionMeshes = collisionMeshes;
            }

            if (dct.ContainsKey("occupancy_zones"))
            {
                var occupancyZones = new List<OccupancyZone>();
                foreach (var o in (List<Dictionary<string, object>>)dct["occupancy_zones"])
                {
                    occupancyZones.Add(OccupancyZone.FromDict(o));
                }

                this.occupancyZones = occupancyZones;
            }

            if (dct.ContainsKey("locations"))
            {
                var locations = new List<Location>();
                foreach (var l in (List<Dictionary<string, object>>)dct["locations"])
                {
                    locations.Add(Location.FromDict(l));
                }
                this.locations = locations;
            }

            if (dct.ContainsKey("trajectories"))
            {
                var trajectories = new List<Trajectory>();
                foreach (var t in (List<Dictionary<string,object>>)dct["trajectories"])
                {
                    trajectories.Add(Trajectory.FromDict(t));
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
            reachSphere.RemoveFromCache();

            foreach (var p in pinchPoints)
            {
                p.RemoveFromCache();
            }

            foreach (var c in collisionMeshes)
            {
                c.RemoveFromCache();
            }

            foreach (var o in occupancyZones)
            {
                o.RemoveFromCache();
            }

            foreach (var l in locations)
            {
                l.RemoveFromCache();
            }

            foreach (var t in trajectories)
            {
                t.RemoveFromCache();
            }

            base.RemoveFromCache();
        }

        public override void AddToCache()
        {
            reachSphere.AddToCache();

            foreach (var p in pinchPoints)
            {
                p.AddToCache();
            }

            foreach (var c in collisionMeshes)
            {
                c.AddToCache();
            }

            foreach (var o in occupancyZones)
            {
                o.AddToCache();
            }

            foreach (var l in locations)
            {
                l.AddToCache();
            }

            foreach (var t in trajectories)
            {
                t.AddToCache();
            }

            base.AddToCache();
        }

        /*
         * Utility Methods
         */

        public override void ChildChangedEvent(List<Dictionary<string, string>> attributeTrace)
        {
            attributeTrace.Add(ChildChangedEventMsg(null, "callback"));
            changesCallback?.Invoke(attributeTrace);
        }

        public override void UpdatedAttribute(string attribute, string verb, string childUuid = null)
        {
            var evnt = new List<Dictionary<string, string>>
                    {
                        ChildChangedEventMsg(attribute, verb, childUuid)
                    };
            changesCallback?.Invoke(evnt);
        }

        /*
         * Update Methods
         */

        public override void DeepUpdate()
        {
            reachSphere.DeepUpdate();

            foreach (var o in occupancyZones)
            {
                o.DeepUpdate();
            }

            foreach (var c in collisionMeshes)
            {
                c.DeepUpdate();
            }

            foreach (var p in pinchPoints)
            {
                p.DeepUpdate();
            }

            foreach (var l in locations)
            {
                l.DeepUpdate();
            }

            foreach (var t in trajectories)
            {
                t.DeepUpdate();
            }

            base.DeepUpdate();

            UpdatedAttribute("occupancy_zones", "update");
            UpdatedAttribute("collision_meshes", "update");
            UpdatedAttribute("pinch_points", "update");
            UpdatedAttribute("reach_sphere", "update");
            UpdatedAttribute("locations", "update");
            UpdatedAttribute("trajectories", "update");
        }

        public override void ShallowUpdate()
        {
            base.ShallowUpdate();

            UpdatedAttribute("occupancy_zones", "update");
            UpdatedAttribute("collision_meshes", "update");
            UpdatedAttribute("pinch_points", "update");
            UpdatedAttribute("reach_sphere", "update");
            UpdatedAttribute("locations", "update");
            UpdatedAttribute("trajectories", "update");
        }
    }
}
