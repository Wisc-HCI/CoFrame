using System.Collections;
using System.Collections.Generic;


namespace EvD
{
    namespace Test 
    {

        [System.Serializable]
        public class Container<T> : Node where T : Node
        {

            /*
            * Private Members
            */

            private string _itemType = null;

            private List<T> _values = null;

            /*
            * Constructors
            */

            public static new string TypeString()
            {
                return string.Format("container<{0}>.", typeof(T).ToString());
            }

            public static new string FullTypeString()
            {
                return Node.FullTypeString() + TypeString();
            }

            public Container(List<T> list = null, string itemType = null, string type = "", string name = "",
                            string uuid = null, Node parent = null, bool appendType = true)
            : base(appendType ? string.Format("container<{0}>.", typeof(T).ToString()) + type : type, name, uuid, parent, appendType)
            {
                values = (list != null) ? list : new List<T>();
                itemType = (itemType != null) ? itemType : typeof(T).ToString();
            }

            public static new Container<T> FromDict(Dictionary<string,object> dct)
            {
                var vals = new List<T>();
                foreach (var v in (List<Dictionary<string,object>>)dct["values"])
                {
                    vals.Add((T)typeof(T).GetMethod("FromDict").Invoke(null, new object[] { v }));
                }

                return new Container<T>(
                    list: vals,
                    itemType: (string)dct["item_type"],
                    type: (string)dct["type"],
                    name: (string)dct["name"],
                    uuid: (string)dct["uuid"],
                    appendType: false
                );
            }

            public override Dictionary<string, object> ToDict()
            {
                var valsDct = new List<Dictionary<string, object>>();
                foreach (var v in values)
                {
                    valsDct.Add(v.ToDict());
                }

                var dct = base.ToDict();
                dct.Add("values", valsDct);
                dct.Add("item_type", itemType);
                return dct;
            }

            /*
            * Accessors and Modifiers
            */

            public string itemType 
            {
                get
                {
                    return _itemType;
                }

                set
                {
                    if (_itemType != value) {
                        _itemType = value;
                        UpdatedAttribute("item_type","set");
                    }
                }
            }

            public List<T> values
            {
                get
                {
                    return _values;
                }

                set
                {
                    if (_values != value)
                    {
                        if (_values != null)
                        {
                            foreach (var v in _values)
                            {
                                v.RemoveFromCache();
                            }
                        }

                        _values = value;
                        foreach (var v in _values)
                        {
                            v.parent = this;
                        }

                        UpdatedAttribute("values", "set");
                    }
                }
            }

            public void Add(T v)
            {
                v.parent = this;
                values.Add(v);
                UpdatedAttribute("values", "add", v.uuid);
            }

            public T Get(string uuid)
            {
                return values[SearchForIndex(uuid)];
            }

            public void Delete(string uuid)
            {
                int idx = SearchForIndex(uuid);
                if (idx == -1)
                {
                    throw new System.Exception("Value not in container");
                }

                values[idx].RemoveFromCache();
                values.RemoveAt(idx);

                UpdatedAttribute("values", "delete", uuid);
            }

            public int SearchForIndex(string uuid)
            {
                int idx = -1;
                for (int i=0; i<values.Count; i++)
                {
                    if (values[i].uuid == uuid)
                    {
                        idx = i;
                        break;
                    }
                }
                return idx;
            }

            public override void Set(Dictionary<string, object> dct)
            {
                if (dct.ContainsKey("values"))
                {
                    var vals = new List<T>();
                    foreach (var v in (List<Dictionary<string,object>>)dct["values"])
                    {
                        vals.Add((T)typeof(T).GetMethod("FromDict").Invoke(null, new object[] { v }));
                    }
                    values = vals;
                }

                if (dct.ContainsValue("item_type")) {
                    itemType = (string)dct["item_type"];
                }

                base.Set(dct);
            }

            /*
            * Cache Methods
            */

            public override void RemoveFromCache()
            {
                foreach (var v in values)
                {
                    v.RemoveFromCache();
                }
                base.RemoveFromCache();
            }

            public override void AddToCache()
            {
                foreach (var v in values)
                {
                    v.AddToCache();
                }
                base.AddToCache();
            }

            /*
            * Children Methods
            */

            public override bool DeleteChild(string uuid)
            {
                bool success = false;

                foreach (var v in values)
                {
                    if (v.uuid == uuid)
                    {
                        Delete(uuid);
                        success = true;
                        break;
                    }
                }

                return success;
            }

            /*
            * Update Methods
            */

            public override void LateConstructUpdate()
            {
                foreach (var v in values)
                {
                    v.LateConstructUpdate();
                }

                base.LateConstructUpdate();
            }

            public override void DeepUpdate()
            {
                foreach (var v in values)
                {
                    v.DeepUpdate();
                }

                base.DeepUpdate();

                UpdatedAttribute("values", "update");
            }

            public override void ShallowUpdate()
            {
                base.ShallowUpdate();

                UpdatedAttribute("values", "update"); 
            }
        }

    }
}
