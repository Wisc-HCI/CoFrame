using System.Collections;
using System.Collections.Generic;


namespace EvD
{
    namespace Data
    {

        [System.Serializable]
        public class Region : Pose
        {
            /*
             * Class Constants
             */

            public static readonly float DEFAULT_ORIENTATION_LIMIT = 1.0f;

            /*
             * Private Members
             */
            
            private bool _freeOrientation;
            private float _uncertaintyOrientationLimit;
            private Orientation _uncertaintyOrientationAltTarget = null;

            /* 
             * Constructors
             */

            public Region(Position centerPosition = null, Orientation centerOrientation = null, bool freeOrientation = true,
                          float uncertaintyOrientationLimit = 1.0f, Orientation uncertaintyOrientationAltTarget = null,
                          string type = "", string name = "", string uuid = null, Node parent = null, bool appendType = true)
            : base(centerPosition,centerOrientation, appendType ? "region." + type : type, name,uuid,parent,appendType)
            {
                this.freeOrientation = freeOrientation;
                this.uncertaintyOrientationLimit = uncertaintyOrientationLimit;
                this.uncertaintyOrientationAltTarget = uncertaintyOrientationAltTarget;
            }

            public static Region FromDct(Dictionary<string,object> dct)
            {
                return new Region(
                    centerPosition: Position.FromDict((Dictionary<string,object>)dct["center_position"]),
                    centerOrientation: Orientation.FromDict((Dictionary<string,object>)dct["center_orientation"]),
                    freeOrientation: (bool)dct["free_orientation"],
                    uncertaintyOrientationLimit: (float)dct["uncertainty_orientation_limit"],
                    uncertaintyOrientationAltTarget: Orientation.FromDict((Dictionary<string,object>)dct["uncertainty_orientation_alt_target"]),
                    type: (string)dct["type"],
                    name: (string)dct["name"],
                    uuid: (string)dct["uuid"],
                    appendType: false
                );
            }

            public override Dictionary<string,object> ToDict() 
            {
                var dct = base.ToDict();
                dct.Remove("position");
                dct.Remove("orientation");
                dct.Add("center_position",position.ToDict());
                dct.Add("center_orientation",orientation.ToDict());
                dct.Add("free_orientation",freeOrientation);
                dct.Add("uncertainty_orientation_limit",uncertaintyOrientationLimit);
                dct.Add("uncertainty_orientation_alt_target",uncertaintyOrientationAltTarget.ToDict());
                return dct;
            }

            /*
            * Accessors and Modifiers
            */

            public Position centerPosition
            {
                get
                {
                    return this.position;
                }

                set
                {
                    this.position = value;
                    UpdatedAttribute("center_position","set");
                }
            }

            public Orientation centerOrientation
            {
                get
                {
                    return this.orientation;
                }

                set
                {
                    this.orientation = value;
                    UpdatedAttribute("center_orientation","set");
                }
            }

            public bool freeOrientation
            {
                get
                {
                    return _freeOrientation;
                }

                set
                {
                    if (_freeOrientation != value) 
                    {
                        _freeOrientation = value;
                        UpdatedAttribute("free_orientation","set");
                    }
                }
            }

            public float uncertaintyOrientationLimit
            {
                get
                {
                    return _uncertaintyOrientationLimit;
                }

                set
                {
                    if (_uncertaintyOrientationLimit != value) 
                    {
                        _uncertaintyOrientationLimit = value;
                        UpdatedAttribute("uncertainty_orientation_limit","set");
                    }
                }
            }

            public Orientation uncertaintyOrientationAltTarget
            {
                get 
                {
                    return _uncertaintyOrientationAltTarget;
                }

                set
                {
                    if (_uncertaintyOrientationAltTarget != value) 
                    {
                        if (_uncertaintyOrientationAltTarget != null)
                        {
                            _uncertaintyOrientationAltTarget.RemoveFromCache();
                        }

                        _uncertaintyOrientationAltTarget = value;
                        if (_uncertaintyOrientationAltTarget != null) 
                        {
                            _uncertaintyOrientationAltTarget.parent = this;
                        }
                        
                        UpdatedAttribute("uncertainty_orientation_alt_target","set");
                    }
                }
            }

            public override void Set(Dictionary<string, object> dct)
            {
                if (dct.ContainsKey("center_position"))
                {
                    centerPosition = Position.FromDict((Dictionary<string,object>)dct["center_position"]);
                }

                if (dct.ContainsKey("center_orientation"))
                {
                    centerOrientation = Orientation.FromDict((Dictionary<string,object>)dct["center_orientation"]);
                }

                if (dct.ContainsKey("free_orientation"))
                {
                    freeOrientation = (bool)dct["free_orientation"];
                }

                if (dct.ContainsKey("uncertainty_orientation_limit"))
                {
                    uncertaintyOrientationLimit = (float)dct["uncertainty_orientation_limit"];
                }

                if (dct.ContainsKey("uncertainty_orientation_alt_target"))
                {
                    uncertaintyOrientationAltTarget = Orientation.FromDict((Dictionary<string,object>)dct["uncertainty_orientation_alt_target"]);
                }

                base.Set(dct);
            }

            /*
             * Cache Methods
             */

            public override void RemoveFromCache()
            {
                if (uncertaintyOrientationAltTarget != null)
                {
                    uncertaintyOrientationAltTarget.RemoveFromCache();
                }
                
                base.RemoveFromCache();
            }

            public override void AddToCache()
            {
                if (uncertaintyOrientationAltTarget != null)
                {
                    uncertaintyOrientationAltTarget.AddToCache();
                }

                base.AddToCache();
            }

            /*
             * Update Methods
             */

            public override void LateConstructUpdate()
            {
                if (uncertaintyOrientationAltTarget != null)
                {
                    uncertaintyOrientationAltTarget.LateConstructUpdate();
                }

                base.LateConstructUpdate();
            }

            public override void DeepUpdate()
            {
                if (uncertaintyOrientationAltTarget != null)
                {
                    uncertaintyOrientationAltTarget.DeepUpdate();
                }

                base.DeepUpdate();

                UpdatedAttribute("center_position","update");
                UpdatedAttribute("center_orientation","update");
                UpdatedAttribute("free_orientation","update");
                UpdatedAttribute("uncertainty_orientation_limit","update");
                UpdatedAttribute("uncertainty_orientation_alt_target","update");
            }

            public override void ShallowUpdate()
            {
                base.ShallowUpdate();

                UpdatedAttribute("center_position","update");
                UpdatedAttribute("center_orientation","update");
                UpdatedAttribute("free_orientation","update");
                UpdatedAttribute("uncertainty_orientation_limit","update");
                UpdatedAttribute("uncertainty_orientation_alt_target","update");
            }
        }

        [System.Serializable]
        public class CubeRegion : Region
        {
            /*
            * Private Members
            */

            private float _uncertaintyX;
            private float _uncertaintyY;
            private float _uncertaintyZ;

            /*
            * Constructors
            */

            public CubeRegion(Position centerPosition = null, Orientation centerOrientation = null, float uncertaintyX = 0.1f,
                              float uncertaintyY = 0.1f, float uncertaintyZ = 0.1f, bool freeOrientation = true, 
                              float uncertaintyOrientationLimit = 1.0f, Orientation uncertaintyOrientationAltTarget = null,
                              string type = "", string name = "", string uuid = null, Node parent = null, bool appendType = true)
            : base(centerPosition, centerOrientation, freeOrientation, uncertaintyOrientationLimit, uncertaintyOrientationAltTarget, 
                   appendType ? "cube-region." + type : type, name,uuid,parent,appendType)
            {
                this.uncertaintyX = uncertaintyX;
                this.uncertaintyY = uncertaintyY;
                this.uncertaintyZ = uncertaintyZ;
            }

            public new static CubeRegion FromDict(Dictionary<string,object> dct)
            {
                return new CubeRegion(
                    centerPosition: Position.FromDict((Dictionary<string,object>)dct["center_position"]),
                    centerOrientation: Orientation.FromDict((Dictionary<string,object>)dct["center_orientation"]),
                    uncertaintyX: (float)dct["uncertainty_x"],
                    uncertaintyY: (float)dct["uncertainty_y"],
                    uncertaintyZ: (float)dct["uncertainty_z"],
                    freeOrientation: (bool)dct["free_orientation"],
                    uncertaintyOrientationLimit: (float)dct["uncertainty_orientation_limit"],
                    uncertaintyOrientationAltTarget: Orientation.FromDict((Dictionary<string,object>)dct["uncertainty_orientation_alt_target"]),
                    type: (string)dct["type"],
                    name: (string)dct["name"],
                    uuid: (string)dct["uuid"],
                    appendType: false
                );
            }

            public override Dictionary<string, object> ToDict()
            {
                var dct = base.ToDict();
                dct.Add("uncertainty_x",uncertaintyX);
                dct.Add("uncertainty_y",uncertaintyY);
                dct.Add("uncertainty_z",uncertaintyZ);
                return dct;
            }

            /*
            * Accessors and Modifiers
            */

            public float uncertaintyX
            {
                get
                {
                    return _uncertaintyX;
                }

                set
                {
                    if (_uncertaintyX != value)
                    {
                        _uncertaintyX = value;
                        UpdatedAttribute("uncertainty_x","set");
                    }
                }
            }

            public float uncertaintyY
            {
                get
                {
                    return _uncertaintyY;
                }

                set
                {
                    if (_uncertaintyY != value)
                    {
                        _uncertaintyY = value;
                        UpdatedAttribute("uncertainty_y","set");
                    }
                }
            }

            public float uncertaintyZ
            {
                get
                {
                    return _uncertaintyZ;
                }

                set
                {
                    if (_uncertaintyZ != value)
                    {
                        _uncertaintyZ = value;
                        UpdatedAttribute("uncertainty_z","set");
                    }
                }
            }

            public override void Set(Dictionary<string, object> dct)
            {
                if (dct.ContainsKey("uncertainty_x"))
                {
                    uncertaintyX = (float)dct["uncertainty_x"];
                }

                if (dct.ContainsKey("uncertainty_y"))
                {
                    uncertaintyY = (float)dct["uncertainty_y"];
                }

                if (dct.ContainsKey("uncertainty_z"))
                {  
                    uncertaintyZ = (float)dct["uncertainty_z"];
                }

                base.Set(dct);
            }

            /*
            * Update Methods
            */

            public override void DeepUpdate()
            {

                base.DeepUpdate();

                UpdatedAttribute("uncertainty_x", "update");
                UpdatedAttribute("uncertainty_y", "update");
                UpdatedAttribute("uncertainty_z", "update");
            }

            public override void ShallowUpdate()
            {
                base.ShallowUpdate();

                UpdatedAttribute("uncertainty_x", "update");
                UpdatedAttribute("uncertainty_y", "update");
                UpdatedAttribute("uncertainty_z", "update");
            }
        }


        [System.Serializable]
        public class SphereRegion : Region
        {
            /*
            * Private Members
            */

            private float _uncertaintyRadius;

            /*
            * Constructors
            */

            public SphereRegion(Position centerPosition = null, Orientation centerOrientation = null, float uncertaintyRadius = 0.1f,
                              bool freeOrientation = true, float uncertaintyOrientationLimit = 1.0f, Orientation uncertaintyOrientationAltTarget = null,
                              string type = "", string name = "", string uuid = null, Node parent = null, bool appendType = true)
            : base(centerPosition, centerOrientation, freeOrientation, uncertaintyOrientationLimit, uncertaintyOrientationAltTarget, 
                   appendType ? "sphere-region." + type : type, name,uuid,parent,appendType)
            {
                this.uncertaintyRadius = uncertaintyRadius;
            }

            public new static SphereRegion FromDict(Dictionary<string,object> dct)
            {
                return new SphereRegion(
                    centerPosition: Position.FromDict((Dictionary<string,object>)dct["center_position"]),
                    centerOrientation: Orientation.FromDict((Dictionary<string,object>)dct["center_orientation"]),
                    uncertaintyRadius: (float)dct["uncertainty_radius"],
                    freeOrientation: (bool)dct["free_orientation"],
                    uncertaintyOrientationLimit: (float)dct["uncertainty_orientation_limit"],
                    uncertaintyOrientationAltTarget: Orientation.FromDict((Dictionary<string,object>)dct["uncertainty_orientation_alt_target"]),
                    type: (string)dct["type"],
                    name: (string)dct["name"],
                    uuid: (string)dct["uuid"],
                    appendType: false
                );
            }

            public override Dictionary<string, object> ToDict()
            {
                var dct = base.ToDict();
                dct.Add("uncertainty_radius",uncertaintyRadius);
                return dct;
            }

            /*
            * Accessors and Modifiers
            */

            public float uncertaintyRadius
            {
                get
                {
                    return _uncertaintyRadius;
                }

                set
                {
                    if (_uncertaintyRadius != value)
                    {
                        _uncertaintyRadius = value;
                        UpdatedAttribute("uncertainty_radius","set");
                    }
                }
            }

            public override void Set(Dictionary<string, object> dct)
            {
                if (dct.ContainsKey("uncertainty_radius"))
                {
                    uncertaintyRadius = (float)dct["uncertainty_radius"];
                }

                base.Set(dct);
            }

            /*
            * Update Methods
            */

            public override void DeepUpdate()
            {

                base.DeepUpdate();

                UpdatedAttribute("uncertainty_radius", "update");
            }

            public override void ShallowUpdate()
            {
                base.ShallowUpdate();

                UpdatedAttribute("uncertainty_radius", "update");
            }
        }
    }
}