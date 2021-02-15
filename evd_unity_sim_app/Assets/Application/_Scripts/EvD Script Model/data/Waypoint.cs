using System.Collections;
using System.Collections.Generic;


namespace EvD
{
    namespace Data
    {

        [System.Serializable]
        public class Waypoint : Pose
        {
            /*
            * Private Members
            */

            private float[] _joints;

            /*
            * Constructors
            */

            public static new string TypeString()
            {
                return "waypoint.";
            }

            public static new string FullTypeString()
            {
                return Pose.FullTypeString() + TypeString();
            }

            public Waypoint(Position position = null, Orientation orientation = null, float[] joints = null, 
                            string type = "", string name = "", string uuid = null, Node parent = null, 
                            bool appendType = true) 
            : base(position,orientation, appendType ? "waypoint." + type : type, name,uuid,parent,appendType)
            {
                this.joints = joints;
            }

            public new static Waypoint FromDict(Dictionary<string,object> dct)
            {
                return new Waypoint(
                    position: Position.FromDict((Dictionary<string, object>)dct["position"]),
                    orientation: Orientation.FromDict((Dictionary<string, object>)dct["orientation"]),
                    joints: (float[])dct["joints"],
                    type: (string)dct["type"],
                    name: (string)dct["name"],
                    uuid: (string)dct["uuid"],
                    appendType: false
                );
            }

            public override Dictionary<string, object> ToDict()
            {
                var dct = base.ToDict();
                dct.Add("joints", joints);
                return dct;
            }

            /*
            * Accessors and Modifiers
            */

            public float[] joints
            {
                get
                {
                    return _joints;
                }

                set
                {
                    if (_joints != value)
                    {
                        _joints = value;
                        UpdatedAttribute("joints", "set");
                    }
                }
            }

            public override void Set(Dictionary<string, object> dct)
            {
                if (dct.ContainsKey("joints"))
                {
                    joints = (float[])dct["joints"];
                }

                base.Set(dct);
            }

            /*
            * Update Methods
            */

            public override void DeepUpdate()
            {

                base.DeepUpdate();

                UpdatedAttribute("joints", "update");
            }

            public override void ShallowUpdate()
            {
                base.ShallowUpdate();

                UpdatedAttribute("joints", "update");
            }
        }

    }
}