using System.Collections;
using System.Collections.Generic;

using EvD.Data;


namespace EvD
{
    namespace Environment
    {

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

            private float _height;
            private float _positionX;
            private float _positionZ;
            private float _scaleX;
            private float _scaleZ;
            private string _occupancyType;

            /*
            * Constructors
            */

            public static new string TypeString()
            {
                return "occupancy-zone.";
            }

            public static new string FullTypeString()
            {
                return Node.FullTypeString() + TypeString();
            }

            public OccupancyZone(string occupancy_type, float posX = 0, float posZ = 0, float sclX = 1, float sclZ = 1, float height = -1,
                                string type = "", string name = "", string uuid = null, Node parent = null, bool appendType = true)
            : base(appendType ? "occupancy-zone." + type : type, name, uuid, parent, appendType)
            {
                this.occupancyType = occupancy_type;

                this.positionX = posX;
                this.positionZ = posZ;
                this.scaleX = sclX;
                this.scaleZ = sclZ;
                this.height = height;
            }

            public new static OccupancyZone FromDict(Dictionary<string, object> dct)
            {
                return new OccupancyZone(
                    occupancy_type: (string)(dct["occupancy_type"]),
                    posX: (float)(dct["position_x"]),
                    posZ: (float)(dct["position_z"]),
                    sclX: (float)(dct["scale_x"]),
                    sclZ: (float)(dct["scale_z"]),
                    height: (float)dct["height"],
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
                dct.Add("height", height);
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

            public float height
            {
                get
                {
                    return _height;
                }

                set
                {
                    if (_height != value)
                    {
                        _height = value;
                        UpdatedAttribute("height","set");
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

                if (dct.ContainsKey("height"))
                {
                    height = (float)dct["height"];
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
                UpdatedAttribute("height","update");
            }

            public override void ShallowUpdate()
            {
                base.ShallowUpdate();

                UpdatedAttribute("occupancy_type", "update");
                UpdatedAttribute("position_x", "update");
                UpdatedAttribute("position_z", "update");
                UpdatedAttribute("scale_x", "update");
                UpdatedAttribute("scale_z", "update");
                UpdatedAttribute("height","update");
            }
        }


    }
}