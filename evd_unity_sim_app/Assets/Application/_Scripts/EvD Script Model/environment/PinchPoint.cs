using System.Collections;
using System.Collections.Generic;

using EvD.Data;


namespace EvD
{
    namespace Environment
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

            private Orientation _axis = null;
            private Position _offset = null;
            private string _link;
            private float _radius;
            private float _length;

            /*
            * Constructors
            */

            public static new string TypeString()
            {
                return "pinch-point.";
            }

            public static new string FullTypeString()
            {
                return Node.FullTypeString() + TypeString();
            }

            public PinchPoint(Orientation axis = null, Position offset = null, string link = "", float radius = 0.05f, float length = 0.2f,
                              string type = "", string name = "", string uuid = null, Node parent = null, bool appendType = true) 
            : base(appendType ? "pinch-point." + type : type, name, uuid, parent, appendType)
            {
                this.axis = (axis != null) ? axis : new Orientation();
                this.offset = (offset != null) ? offset : new Position();
                this.link = link;
                this.radius = radius;
                this.length = length;
            }

            public new static PinchPoint FromDict(Dictionary<string,object> dct)
            {
                return new PinchPoint(
                    axis: Orientation.FromDict((Dictionary<string,object>)dct["axis"]),
                    offset: Position.FromDict((Dictionary<string,object>)dct["offset"]),
                    link: (string)dct["link"],
                    radius: (float)dct["radius"],
                    length: (float)dct["length"],
                    type: (string)(dct["type"]),
                    name: (string)(dct["name"]),
                    uuid: (string)(dct["uuid"]),
                    appendType: false
                );
            }

            public override Dictionary<string,object> ToDict()
            {
                var dct = base.ToDict();
                dct.Add("axis",axis.ToDict());
                dct.Add("offset",offset.ToDict());
                dct.Add("link",link);
                dct.Add("radius",radius);
                dct.Add("length",length);
                return dct;
            }

            /*
            * Accessors and Modifiers
            */

            public Orientation axis
            {
                get
                {
                    return _axis;
                }

                set
                {
                    if (_axis != value)
                    {
                        if (_axis != null)
                        {
                            _axis.RemoveFromCache();
                        }

                        _axis = value;
                        _axis.parent = this;
                        UpdatedAttribute("axis","set");
                    }
                }
            }
            
            public Position offset
            {
                get
                {
                    return _offset;
                }

                set
                {
                    if (_offset != value)
                    {
                        if (_offset != null)
                        {
                            _offset.RemoveFromCache();
                        }

                        _offset = value;
                        _offset.parent = this;
                        UpdatedAttribute("offset","set");
                    }
                }
            }
            
            public string link
            {
                get
                {
                    return _link;
                }

                set
                {
                    if (_link != value)
                    {
                        _link = value;
                        UpdatedAttribute("link","set");
                    }
                }
            }
            
            public float radius
            {
                get
                {
                    return _radius;
                }

                set
                {
                    if (_radius != value)
                    {
                        _radius = value;
                        UpdatedAttribute("radius","set");
                    }
                }
            }
            
            public float length
            {
                get
                {
                    return _length;
                }

                set
                {
                    if (_length != value)
                    {
                        _length = value;
                        UpdatedAttribute("length","set");
                    }
                }
            }

            public override void Set(Dictionary<string, object> dct)
            {
                if (dct.ContainsKey("axis"))
                {
                    axis = Orientation.FromDict((Dictionary<string,object>)dct["axis"]);
                }

                if (dct.ContainsKey("offset"))
                {
                    offset = Position.FromDict((Dictionary<string,object>)dct["offset"]);
                }

                if (dct.ContainsKey("link"))
                {
                    link = (string)dct["link"];
                }

                if (dct.ContainsKey("radius"))
                {
                    radius = (float)dct["radius"];
                }

                if (dct.ContainsKey("length"))
                {
                    length = (float)dct["length"];
                }

                base.Set(dct);
            }

            /*
            * Cache methods
            */
            
            public override void RemoveFromCache()
            {
                axis.RemoveFromCache();
                offset.RemoveFromCache();

                base.RemoveFromCache();
            }

            public override void AddToCache()
            {
                axis.AddToCache();
                offset.AddToCache();

                base.AddToCache();
            }

            /*
            * Update Methods
            */

            public override void LateConstructUpdate()
            {
                axis.LateConstructUpdate();
                offset.LateConstructUpdate();

                base.LateConstructUpdate();
            }

            public override void DeepUpdate()
            {
                axis.DeepUpdate();
                offset.DeepUpdate();
                
                base.DeepUpdate();

                UpdatedAttribute("axis","update");
                UpdatedAttribute("offset","update");
                UpdatedAttribute("link","update");
                UpdatedAttribute("radius","update");
                UpdatedAttribute("length","update");
            }

            public override void ShallowUpdate()
            {
                base.ShallowUpdate();

                UpdatedAttribute("axis","update");
                UpdatedAttribute("offset","update");
                UpdatedAttribute("link","update");
                UpdatedAttribute("radius","update");
                UpdatedAttribute("length","update");
            }
        }

    }
}