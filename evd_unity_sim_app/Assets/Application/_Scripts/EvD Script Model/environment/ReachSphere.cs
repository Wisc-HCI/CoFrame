using System.Collections;
using System.Collections.Generic;

using EvD.Data;


namespace EvD
{
    namespace Environment
    {

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

            private float _radius;
            private Position _offset = null;

            /*
            * Constructors
            */

            public ReachSphere(float radius = 0.5f, Position offset = null, string type = "", string name = "", 
                               string uuid = null, Node parent = null, bool appendType = true)
            : base(appendType ? "reach-sphere." + type : type, name, uuid, parent, appendType)
            {
                this.radius = radius;
                this.offset = (offset != null) ? offset : new Position();
            }

            public new static ReachSphere FromDict(Dictionary<string, object> dct)
            {
                return new ReachSphere(
                    radius: (float)dct["radius"],
                    offset: Position.FromDict((Dictionary<string,object>)dct["offset"]),
                    type: (string)(dct["type"]),
                    name: (string)(dct["name"]),
                    uuid: (string)(dct["uuid"])
                );
            }

            public override Dictionary<string, object> ToDict()
            {
                var dct = base.ToDict();
                dct.Add("radius",radius);
                dct.Add("offset",offset.ToDict());
                return dct;
            }

            /*
            * Accessors and Modifiers
            */

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

            public override void Set(Dictionary<string, object> dct)
            {
                if (dct.ContainsKey("radius"))
                {
                    radius = (float)dct["radius"];
                }

                if (dct.ContainsKey("offset"))
                {
                    offset = Position.FromDict((Dictionary<string,object>)dct["offset"]);
                }

                base.Set(dct);
            }

            /*
            * Cache methods
            */

            public override void RemoveFromCache()
            {
                offset.RemoveFromCache();

                base.RemoveFromCache();
            }

            public override void AddToCache()
            {
                offset.AddToCache();

                base.AddToCache();
            }

            /*
            * Update Methods
            */

            public override void LateConstructUpdate()
            {
                offset.LateConstructUpdate();

                base.LateConstructUpdate();
            }

            public override void DeepUpdate()
            {
                offset.DeepUpdate();

                base.DeepUpdate();

                UpdatedAttribute("offset","update");
                UpdatedAttribute("radius","update");
            }

            public override void ShallowUpdate()
            {
                base.ShallowUpdate();

                UpdatedAttribute("offset","update");
                UpdatedAttribute("radius","update");
            }
        }
 

    }
}