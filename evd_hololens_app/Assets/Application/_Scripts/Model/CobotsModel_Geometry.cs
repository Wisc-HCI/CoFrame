using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Cobots
{
    public class Pose : Node
    {
        /*
         * Private Members
         */

        private Position _position = null;
        private Orientation _orientation = null;

        /*
         * Constructors
         */

        public Pose(Position position = null, Orientation orientation = null, string type = "", string name = "", 
                    string uuid = null, Node parent = null, bool appendType = true) 
        : base(appendType ? "pose." + type : type, name, uuid, parent, appendType)
        {
            this.position = (position == null) ? new Position(parent: this) : position;
            this.orientation = (orientation == null) ? new Orientation(parent: this) : orientation;
        }

        public static Pose FromUnity(Transform tf)
        {
            return new Pose(Position.FromUnity(tf.position),Orientation.FromUnity(Quaternion.Euler(tf.eulerAngles)));
        }

        public static Pose FromRos(RosSharp.RosBridgeClient.Messages.Geometry.Pose p)
        {
            return new Pose(Position.FromRos(p.position), Orientation.FromRos(p.orientation));
        }

        public RosSharp.RosBridgeClient.Messages.Geometry.Pose ToRos()
        {
            var msg = new RosSharp.RosBridgeClient.Messages.Geometry.Pose();
            msg.position = position.ToRos();
            msg.orientation = orientation.ToRos();
            return msg;
        }

        public new static Pose FromDict(Dictionary<string,object> dct)
        {
            return new Pose(
                position: Position.FromDict((Dictionary<string,object>)dct["position"]),
                orientation: Orientation.FromDict((Dictionary<string, object>)dct["orientation"]),
                type: dct.ContainsKey("type") ? (string)dct["type"] : "",
                name: dct.ContainsKey("name") ? (string)dct["name"] : "",
                uuid: dct.ContainsKey("uuid") ? (string)dct["uuid"] : null,
                appendType: !dct.ContainsKey("type")
            );
        }

        public override Dictionary<string,object> ToDict()
        {
            var dct = base.ToDict();
            dct.Add("position", position.ToDict());
            dct.Add("orientation", orientation.ToDict());
            return dct;
        }

        /*
         * Accessors and Modifiers
         */

        public Position position
        {
            get
            {
                return _position;
            }

            set
            {
                if (_position != value)
                {
                    _position = value;
                    _position.parent = this;
                    UpdatedAttribute("position", "set");
                }
            }
        }

        public Orientation orientation
        {
            get
            {
                return _orientation;
            }

            set
            {
                if (_orientation != value)
                {
                    _orientation = value;
                    _orientation.parent = this;
                    UpdatedAttribute("orientation", "set");
                }
            }
        }

        public override void Set(Dictionary<string,object> dct)
        {
            if (dct.ContainsKey("position"))
            {
                position = Position.FromDict((Dictionary<string,object>)dct["position"]);
            }

            if (dct.ContainsKey("orientation"))
            {
                orientation = Orientation.FromDict((Dictionary<string, object>)dct["orientation"]);
            }

            base.Set(dct);
        }

        /*
         * Update Methods
         */

        public override void DeepUpdate()
        {
            orientation.DeepUpdate();
            position.DeepUpdate();

            base.DeepUpdate();

            UpdatedAttribute("position", "update");
            UpdatedAttribute("orientation", "update");
        }

        public override void ShallowUpdate()
        {
            base.ShallowUpdate();

            UpdatedAttribute("position", "update");
            UpdatedAttribute("orientation", "update");
        }
    }

    public class Position : Node
    {
        /*
         * Private Members
         */

        private float _x = 0;
        private float _y = 0;
        private float _z = 0;

        /*
         * Constructors
         */

        public Position(float x = 0, float y = 0, float z = 0, bool inRosForm = true, string type = "",
                        string name = "", string uuid = null, Node parent = null, bool appendType = true)
        : base(appendType ? "position." + type : type, name, uuid, parent, appendType)
        {
            if (inRosForm)
            {
                this.x = x;
                this.y = y;
                this.z = z;
            }
            else // convert this quaternion to ROS space
            {
                var p = RosSharp.TransformExtensions.Unity2Ros(new Vector3(x, y, z));
                this.x = p.x;
                this.y = p.y;
                this.z = p.z;
            }
        }

        public new static Position FromDict(Dictionary<string, object> dct)
        {
            return new Position(
                x: (float)dct["x"],
                y: (float)dct["y"],
                z: (float)dct["z"],
                inRosForm: true,
                type: dct.ContainsKey("type") ? (string)dct["type"] : "",
                name: dct.ContainsKey("name") ? (string)dct["name"] : "",
                uuid: dct.ContainsKey("uuid") ? (string)dct["uuid"] : null,
                appendType: !dct.ContainsKey("type")
            );
        }

        public static Position GenerateRandom(Vector3 min, Vector3 max)
        {
            return new Position(Random.Range(min.x, max.x), Random.Range(min.y, max.y), Random.Range(min.z, max.z));
        }

        public static Position FromUnity(Vector3 v)
        {
            return new Position(v.x, v.y, v.z, false);
        }

        public static Position FromRos(RosSharp.RosBridgeClient.Messages.Geometry.Point p)
        {
            return new Position(p.x, p.y, p.z);
        }

        public override Dictionary<string, object> ToDict()
        {
            var dct = base.ToDict();
            dct.Add("x", x);
            dct.Add("y", y);
            dct.Add("z", z);
            return dct;
        }

        public Vector3 ToUnity()
        {
            return RosSharp.TransformExtensions.Ros2Unity(new Vector3(x, y, z));
        }

        public RosSharp.RosBridgeClient.Messages.Geometry.Point ToRos()
        {
            var msg = new RosSharp.RosBridgeClient.Messages.Geometry.Point();
            msg.x = x;
            msg.y = y;
            msg.z = z;
            return msg;
        }

        /*
         * Accessors and Modifiers
         */

        public float x
        {
            get
            {
                return _x;
            }

            set
            {
                if (_x != value)
                {
                    _x = value;
                    UpdatedAttribute("x", "set");
                }
            }
        }

        public float y
        {
            get
            {
                return _y;
            }

            set
            {
                if (_y != value)
                {
                    _y = value;
                    UpdatedAttribute("y", "set");
                }
            }
        }

        public float z
        {
            get
            {
                return _z;
            }

            set
            {
                if (_z != value)
                {
                    _z = value;
                    UpdatedAttribute("z", "set");
                }
            }
        }

        public override void Set(Dictionary<string, object> dct)
        {
            if (dct.ContainsKey("x"))
            {
                x = (float)dct["x"];
            }

            if (dct.ContainsKey("y"))
            {
                y = (float)dct["y"];
            }

            if (dct.ContainsKey("z"))
            {
                z = (float)dct["z"];
            }

            base.Set(dct);
        }

        /*
         * Update Methods
         */

        public override void DeepUpdate()
        {
            base.DeepUpdate();

            UpdatedAttribute("x", "update");
            UpdatedAttribute("y", "update");
            UpdatedAttribute("z", "update");
        }

        public override void ShallowUpdate()
        {
            base.ShallowUpdate();

            UpdatedAttribute("x", "update");
            UpdatedAttribute("y", "update");
            UpdatedAttribute("z", "update");
        }
    }

    public class Orientation : Node
    {

        /*
         * Private Members
         */

        private float _x = 0;
        private float _y = 0;
        private float _z = 0;
        private float _w = 0;

        /*
         * Constructors
         */

        public static Orientation Identity()
        {
            return new Orientation();
        }

        public Orientation(float x=0, float y=0, float z=0, float w=1, bool inRosForm = true, string type = "", 
                           string name = "", string uuid = null, Node parent = null, bool appendType = true) 
        : base(appendType? "orientation."+type : type, name,uuid,parent,appendType)
        {
            if (inRosForm)
            {
                this.x = x;
                this.y = y;
                this.z = z;
                this.w = w;
            }
            else // convert this quaternion to ROS space
            {
                var qr = RosSharp.TransformExtensions.Unity2Ros(new Quaternion(x,y,z,w));
                this.x = qr.x;
                this.y = qr.y;
                this.z = qr.z;
                this.w = qr.w;
            }
        }

        public new static Orientation FromDict(Dictionary<string, object> dct)
        {
            return new Orientation(
                x: (float)dct["x"],
                y: (float)dct["y"],
                z: (float)dct["z"],
                w: (float)dct["w"],
                inRosForm: true,
                type: dct.ContainsKey("type") ? (string)dct["type"] : "",
                name: dct.ContainsKey("name") ? (string)dct["name"] : "",
                uuid: dct.ContainsKey("uuid") ? (string)dct["uuid"] : null,
                appendType: !dct.ContainsKey("type")
            );
        }

        public static Orientation GenerateRandom(Vector3 min, Vector3 max)
        {
            var euler = new Vector3(Random.Range(min.x, max.x), Random.Range(min.y, max.y), Random.Range(min.z, max.z));
            return FromUnity(Quaternion.Euler(euler));
        }

        public static Orientation FromUnity(Quaternion q)
        {
            return new Orientation(q.x,q.y,q.z,q.w,false);
        }

        public static Orientation FromRos(RosSharp.RosBridgeClient.Messages.Geometry.Quaternion q)
        {
            return new Orientation(q.x, q.y, q.z, q.w);
        }

        public override Dictionary<string, object> ToDict()
        {
            var dct = base.ToDict();
            dct.Add("x", x);
            dct.Add("y", y);
            dct.Add("z", z);
            dct.Add("w", w);
            return dct;
        }

        public Quaternion ToUnity()
        {
            return RosSharp.TransformExtensions.Ros2Unity(new Quaternion(x, y, z, w));
        }

        public RosSharp.RosBridgeClient.Messages.Geometry.Quaternion ToRos()
        {
            var msg = new RosSharp.RosBridgeClient.Messages.Geometry.Quaternion();
            msg.x = x;
            msg.y = y;
            msg.z = z;
            msg.w = w;
            return msg;
        }

        /*
         * Accessors and Modifiers
         */

        public float x
        {
            get
            {
                return _x;
            }

            set
            {
                if (_x != value)
                {
                    _x = value;
                    UpdatedAttribute("x", "set");
                }
            }
        }

        public float y
        {
            get
            {
                return _y;
            }

            set
            {
                if (_y != value)
                {
                    _y = value;
                    UpdatedAttribute("y", "set");
                }
            }
        }

        public float z
        {
            get
            {
                return _z;
            }

            set
            {
                if (_z != value)
                {
                    _z = value;
                    UpdatedAttribute("z", "set");
                }
            }
        }

        public float w
        {
            get
            {
                return _w;
            }

            set
            {
                if (_w != value)
                {
                    _w = value;
                    UpdatedAttribute("w", "set");
                }
            }
        }

        public override void Set(Dictionary<string, object> dct)
        {
            if (dct.ContainsKey("x"))
            {
                x = (float)dct["x"];
            }

            if (dct.ContainsKey("y"))
            {
                y = (float)dct["y"];
            }

            if (dct.ContainsKey("z"))
            {
                z = (float)dct["z"];
            }

            if (dct.ContainsKey("w"))
            {
                w = (float)dct["w"];
            }

            base.Set(dct);
        }

        /*
         * Update Methods
         */

        public override void DeepUpdate()
        {
            base.DeepUpdate();

            UpdatedAttribute("x", "update");
            UpdatedAttribute("y", "update");
            UpdatedAttribute("z", "update");
            UpdatedAttribute("w", "update");
        }

        public override void ShallowUpdate()
        {
            base.ShallowUpdate();

            UpdatedAttribute("x", "update");
            UpdatedAttribute("y", "update");
            UpdatedAttribute("z", "update");
            UpdatedAttribute("w", "update");
        }
    }
}
