using System.Collections;
using System.Collections.Generic;

using EvD.Data;


namespace EvD
{
    namespace Environment
    {

        [System.Serializable]
        public class EnvironmentContext : Context
        {
            /*
            * Attributes
            */
            
            private ReachSphere _reachSphere = null;
            private List<PinchPoint> _pinchPoints = null;
            private List<CollisionMesh> _collisionMeshes = null;
            private List<OccupancyZone> _occupancyZones = null;

            /*
            * Constructors
            */

            public EnvironmentContext(ReachSphere reachSphere, List<PinchPoint> pinchPoints, List<CollisionMesh> collisionMeshes,
                               List<OccupancyZone> occupancyZones, List<Location> locations, List<Machine> machines, List<Thing> things, 
                               List<Trajectory> trajectories, List<Waypoint> waypoints,
                               string type = "", string name = "", string uuid = null, Node parent = null, bool appendType = true)
            : base(locations, machines, things, waypoints, trajectories, appendType ? "environment." + type : type, name, uuid, parent, appendType)
            {
                this.reachSphere = reachSphere;
                this.pinchPoints = pinchPoints;
                this.collisionMeshes = collisionMeshes;
                this.occupancyZones = occupancyZones;
            }

            public static EnvironmentContext FromDict(Dictionary<string, object> dct, System.Action<List<Dictionary<string, string>>> changesCallback = null)
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

                return new EnvironmentContext(
                    reachSphere: ReachSphere.FromDict((Dictionary<string,object>)dct["reach_sphere"]),
                    pinchPoints: pinchPoints,
                    collisionMeshes: collisionMeshes,
                    occupancyZones: occupancyZones,
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

                var dct = base.ToDict();
                dct.Add("reach_sphere", reachSphere.ToDict());
                dct.Add("pinch_points", pinchDct);
                dct.Add("collision_meshes", collisionDct);
                dct.Add("occupancy_zones", occupancyDct);
                return dct;
            }

            /*
            * Accessors and Modifiers
            */

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

                base.Set(dct);
            }

            /*
            * Cache Methods
            */

            public override void RemoveFromCache()
            {
                reachSphere.RemoveFromCache();

                foreach (var o in occupancyZones)
                {
                    o.RemoveFromCache();
                }

                foreach (var c in collisionMeshes)
                {
                    c.RemoveFromCache();
                }

                foreach (var p in pinchPoints)
                {
                    p.RemoveFromCache();
                }

                base.RemoveFromCache();
            }

            public override void AddToCache()
            {
                reachSphere.AddToCache();

                foreach (var o in occupancyZones)
                {
                    o.AddToCache();
                }

                foreach (var c in collisionMeshes)
                {
                    c.AddToCache();
                }

                foreach (var p in pinchPoints)
                {
                    p.AddToCache();
                }

                base.AddToCache();
            }

            /*
            * Update Methods
            */

            public override void LateConstructUpdate()
            {
                reachSphere.LateConstructUpdate();

                foreach (var o in occupancyZones)
                {
                    o.LateConstructUpdate();
                }

                foreach (var c in collisionMeshes)
                {
                    c.LateConstructUpdate();
                }

                foreach (var p in pinchPoints)
                {
                    p.LateConstructUpdate();
                }

                base.LateConstructUpdate();
            }

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

                base.DeepUpdate();

                UpdatedAttribute("occupancy_zones", "update");
                UpdatedAttribute("collision_meshes", "update");
                UpdatedAttribute("pinch_points", "update");
                UpdatedAttribute("reach_sphere", "update");
            }

            public override void ShallowUpdate()
            {
                base.ShallowUpdate();

                UpdatedAttribute("occupancy_zones", "update");
                UpdatedAttribute("collision_meshes", "update");
                UpdatedAttribute("pinch_points", "update");
                UpdatedAttribute("reach_sphere", "update");
            }
        }


    }
}