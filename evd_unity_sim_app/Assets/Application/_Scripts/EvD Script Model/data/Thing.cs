using System.Collections;
using System.Collections.Generic;


namespace EvD
{
    namespace Data
    {

        [System.Serializable]
        public class Thing : Node
        {
            /*
            * Private Members
            */

            private string _thingType;
            private int _safetyLevel;
            private string _meshId;
            private float _weight;

            /*
            * Constructors
            */

            public Thing(string thingType, int safetyLevel, string meshId, float weight, string type = "", 
                         string name = "", string uuid = null, Node parent = null, bool appendType = true )
            : base(appendType ? "thing." + type : type, name, uuid, parent, appendType)
            {
                this.thingType = thingType;
                this.safetyLevel = safetyLevel;
                this.meshId = meshId;
                this.weight = weight;
            }

            public new static Thing FromDict(Dictionary<string,object> dct)
            {
                return new Thing(
                    thingType: (string)dct["thing_type"],
                    safetyLevel: (int)dct["safety_level"],
                    meshId: (string)dct["mesh_id"],
                    weight: (float)dct["weight"],
                    type: dct.ContainsKey("type") ? (string)dct["type"] : "",
                    name: dct.ContainsKey("name") ? (string)dct["name"] : "",
                    uuid: dct.ContainsKey("uuid") ? (string)dct["uuid"] : null,
                    appendType: !dct.ContainsKey("type")
                );
            }

            public override Dictionary<string,object> ToDict() 
            {
                var dct = base.ToDict();
                dct.Add("thing_type", thingType);
                dct.Add("safety_level",safetyLevel);
                dct.Add("mesh_id",meshId);
                dct.Add("weight",weight);
                return dct;
            }

            /*
            * Accessors and Modifiers
            */

            public string thingType
            {
                get
                {
                    return _thingType;
                }

                set
                {
                    if (_thingType != value)
                    {
                        _thingType = value;
                        UpdatedAttribute("thing_type","set");
                    }
                }
            }

            public int safetyLevel
            {
                get
                {
                    return _safetyLevel;
                }

                set
                {
                    if (_safetyLevel != value)
                    {
                        _safetyLevel = value;
                        UpdatedAttribute("safety_level","set");
                    }
                }
            }

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

            public float weight
            {
                get
                {
                    return _weight;
                }

                set
                {
                    if (_weight != value)
                    {
                        _weight = value;
                        UpdatedAttribute("weight","set");
                    }
                }
            }

            public override void Set(Dictionary<string, object> dct)
            {
                if (dct.ContainsKey("thing_type"))
                {
                    thingType = (string)dct["thing_type"];
                }

                if (dct.ContainsKey("safety_level"))
                {
                    safetyLevel = (int)dct["safety_level"];
                }

                if (dct.ContainsKey("mesh_id"))
                {
                    meshId = (string)dct["mesh_id"];
                }

                if (dct.ContainsKey("weight"))
                {
                    weight = (float)dct["weight"];
                }

                base.Set(dct);
            }

            /*
            * Update Methods
            */

             public override void DeepUpdate()
            {
                base.DeepUpdate();

                UpdatedAttribute("thing_type","update");
                UpdatedAttribute("safety_level","update");
                UpdatedAttribute("mesh_id","update");
                UpdatedAttribute("weight","update");
            }

            public override void ShallowUpdate()
            {
                base.ShallowUpdate();

                UpdatedAttribute("thing_type","update");
                UpdatedAttribute("safety_level","update");
                UpdatedAttribute("mesh_id","update");
                UpdatedAttribute("weight","update");
            }

        }

    }
}