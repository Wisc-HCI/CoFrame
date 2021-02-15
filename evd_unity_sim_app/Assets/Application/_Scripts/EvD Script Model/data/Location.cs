using System.Collections;
using System.Collections.Generic;


namespace EvD
{
    namespace Data
    {

        [System.Serializable]
        public class Location : Waypoint
        {

            /*
            * Constructors
            */
            
            public static new string TypeString()
            {
                return "location.";
            }

            public static new string FullTypeString()
            {
                return Waypoint.FullTypeString() + TypeString();
            }
            
            public Location(Position position = null, Orientation orientation = null, float[] joints = null, 
                            string type="", string name = "", string uuid = null, Node parent = null, 
                            bool appendType=true) 
            : base(position,orientation,joints, appendType ? "location." + type : type, name, uuid,parent,appendType) { }

            public new static Location FromDict(Dictionary<string, object> dct)
            {
                return new Location(
                    position: Position.FromDict((Dictionary<string, object>)dct["position"]),
                    orientation: Orientation.FromDict((Dictionary<string, object>)dct["orientation"]),
                    joints: (float[])dct["joints"],
                    type: (string)dct["type"],
                    name: (string)dct["name"],
                    uuid: (string)dct["uuid"],
                    appendType: false
                );
            }
        }

    }
}