using System.Collections;
using System.Collections.Generic;

using EvD.Data;


namespace EvD
{
    namespace Environment
    {

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

            private string _meshId;
            private string _link;
            private Pose _poseOffset = null;

            /*
            * Constructors
            */

            public CollisionMesh(string meshId = "", Pose poseOffset = null, string link = "app", string type = "", 
                                 string name = "", string uuid = null, Node parent = null, bool appendType = true)
            : base(appendType ? "collision-mesh." + type : type, name, uuid, parent, appendType)
            {
                this.meshId = meshId;
                this.poseOffset = (poseOffset != null) ? poseOffset : new Pose();
                this.link = link;
            }

            public new static CollisionMesh FromDict(Dictionary<string, object> dct)
            {
                return new CollisionMesh(
                    meshId: (string)dct["mesh_id"],
                    poseOffset: Pose.FromDict((Dictionary<string,object>)dct["pose_offset"]),
                    link: (string)dct["link"],
                    type: (string)(dct["type"]),
                    name: (string)(dct["name"]),
                    uuid: (string)(dct["uuid"])
                );
            }

            public override Dictionary<string, object> ToDict()
            {
                var dct = base.ToDict();
                dct.Add("mesh_id",meshId);
                dct.Add("pose_offset",poseOffset.ToDict());
                dct.Add("link",link);
                return dct;
            }

            /*
            * Accessors and Modifiers
            */

            public string meshId
            {
                get
                {
                    return _meshId;
                }

                set
                {
                    if (_meshId != value)
                    {
                        _meshId = value;
                        UpdatedAttribute("mesh_id","set");
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

            public Pose poseOffset
            {
                get
                {
                    return _poseOffset;
                }

                set
                {
                    if (_poseOffset != value) 
                    {
                        if (_poseOffset != null) 
                        {
                            _poseOffset.RemoveFromCache();
                        }

                        _poseOffset = value;
                        _poseOffset.parent = this;
                        UpdatedAttribute("pose_offset","set");
                    }
                }
            }

            public override void Set(Dictionary<string, object> dct)
            {
                if (dct.ContainsKey("mesh_id"))
                {
                    meshId = (string)dct["mesh_id"];
                }

                if (dct.ContainsKey("pose_offset"))
                {
                    poseOffset = Pose.FromDict((Dictionary<string,object>)dct["pose_offset"]);
                }

                if (dct.ContainsKey("link"))
                {
                    link = (string)dct["link"];
                }

                base.Set(dct);
            }

            /*
            * Cache methods
            */

            public override void RemoveFromCache()
            {
                poseOffset.RemoveFromCache();

                base.RemoveFromCache();
            }

            public override void AddToCache()
            {
                poseOffset.AddToCache();

                base.AddToCache();
            }

            /*
            * Update Methods
            */

            public override void LateConstructUpdate()
            {
                poseOffset.LateConstructUpdate();

                base.LateConstructUpdate();
            }

            public override void DeepUpdate()
            {
                poseOffset.DeepUpdate();

                base.DeepUpdate();

                UpdatedAttribute("mesh_id","update");
                UpdatedAttribute("pose_offset","update");
                UpdatedAttribute("link","update");
            }

            public override void ShallowUpdate()
            {
                base.ShallowUpdate();

                UpdatedAttribute("mesh_id","update");
                UpdatedAttribute("pose_offset","update");
                UpdatedAttribute("link","update");
            }
        }


    }
}