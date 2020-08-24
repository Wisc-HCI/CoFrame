using System.Collections;
using System.Collections.Generic;

namespace Cobots
{

    [System.Serializable]
    public class Node
    {

        /*
         * Private Members
         */

        private string _name = null;
        private string _type = null;
        private string _uuid = null;
        private Node _parent = null;

        /*
         * Constructors
         */

        public Node(string type = "", string name = "", string uuid = null, Node parent = null, bool appendType = true)
        {
            this.uuid = (uuid == null) ? GenerateUuid(type) : uuid;
            this.parent = parent;
            this.type = appendType ? "node." + type : type;
            this.name = name;
        }

        public static Node FromDict(Dictionary<string, object> dct)
        {
            return new Node(
                type: dct.ContainsKey("type") ? (string)dct["type"] : "",
                name: dct.ContainsKey("name") ? (string)dct["name"] : "",
                uuid: dct.ContainsKey("uuid") ? (string)dct["uuid"] : null,
                appendType: !dct.ContainsKey("type")
            );
        }

        public virtual Dictionary<string, object> ToDict()
        {
            var dct = new Dictionary<string, object>
            {
                { "type", type },
                { "name", name },
                { "uuid", uuid }
            };
            return dct;
        }

        /*
         * Accessors and Modifiers
         */

        public virtual Context context
        {
            get
            {
                return (_parent != null) ? _parent.context : null; ;
            }

            set
            {
                throw new System.Exception("Type does not support setting context");
            }
        }

        public string name
        {
            get
            {
                return _name;
            }

            set
            {
                if (_name != value)
                {
                    _name = value;
                    UpdatedAttribute("name", "set");
                }
            }
        }

        public string type
        {
            get
            {
                return _type;
            }

            set
            {
                if (_type != value)
                {
                    _type = value;
                    UpdatedAttribute("type", "set");
                }
            }
        }

        public string uuid
        {
            get
            {
                return _uuid;
            }

            protected set
            {
                _uuid = value;
            }
        }

        public Node parent
        {
            get
            {
                return _parent;
            }

            set
            {
                if (_parent != value)
                {
                    RemoveFromCache();
                    _parent = value;
                    AddToCache();

                    UpdatedAttribute("parent", "set");
                }
            }
        }

        public virtual void Set(Dictionary<string, object> dct)
        {
            // Note: cannot set uuid

            if (dct.ContainsKey("name"))
            {
                name = (string)dct["name"];
            }

            if (dct.ContainsKey("type"))
            {
                type = (string)dct["type"];
            }
        }

        /*
         * Cache Methods
         */

        public virtual Cache cache {

            get
            {
                return (parent != null) ? _parent.cache : null;
            }
            
            set
            {
                throw new System.Exception("Type does not support setting cache");
            }

        }

        public virtual void RemoveFromCache()
        {
            if (cache != null)
            {
                cache.Remove(uuid);
            }
        }

        public virtual void AddToCache()
        {
            if (cache != null)
            {
                cache.Add(uuid, this);
            }
        }

        public virtual void RefreshCacheEntry()
        {
            RemoveFromCache();
            AddToCache();
        }

        /*
         * Children Methods (Optional)
         */

        public virtual bool DeleteChild(string uuid)
        {
            // Write this for each sub-node type that has children
            return true;
        }

        public virtual void ChildChangedEvent(List<Dictionary<string,string>> attributeTrace)
        {
            if (parent != null)
            {
                attributeTrace.Add(ChildChangedEventMsg(null, "callback"));
                parent.ChildChangedEvent(attributeTrace);
            }
        }

        /*
         * Utility Methods
         */

        public static string GenerateUuid(string type)
        {
            return type + "-ar-" + System.Guid.NewGuid().ToString();
        }

        public Dictionary<string,string> ChildChangedEventMsg(string attribute, string verb, string childUuid = null)
        {
            var dct = new Dictionary<string, string>
            {
                { "type", type },
                { "uuid", uuid },
                { "attribute", attribute },
                { "verb", verb },
                { "child_uuid", childUuid }
            };
            return dct;
        }

        public override bool Equals(object obj)
        {
            if (obj is Node)
            {
                return this.Equals((Node)obj);
            }
            return false;
        }

        public bool Equals(Node n)
        {
            if ((object)(n) == null)
            {
                return false;
            } 
            else
            {
                return n.uuid == uuid;
            }
        }

        public static bool operator ==(Node lhs, Node rhs)
        {
            if ((object)(lhs) == null && (object)(rhs) == null)
            {
                return true;
            }
            else if ((object)(lhs) == null)
            {
                return false;
            }
            else
            {
                return lhs.Equals(rhs);
            }
        }

        public static bool operator !=(Node lhs, Node rhs)
        {
            if ((object)(lhs) == null && (object)(rhs) == null)
            {
                return false;
            }
            else if ((object)(lhs) == null)
            {
                return true;
            }
            else
            {
                return !(lhs.Equals(rhs));
            }
        }

        public override int GetHashCode()
        {
            return uuid.GetHashCode();
        }

        public virtual void UpdatedAttribute(string attribute, string verb, string childUuid = null)
        {
            if (parent != null)
            {
                var evnt = new List<Dictionary<string, string>>
                        {
                            ChildChangedEventMsg(attribute, verb, childUuid)
                        };
                parent.ChildChangedEvent(evnt);
            }
        }

        /*
         * Update Methods
         */

        public virtual void DeepUpdate()
        {
            UpdatedAttribute("name", "update");
            UpdatedAttribute("type", "update");
            UpdatedAttribute("uuid", "update");
        }

        public virtual void ShallowUpdate()
        {
            UpdatedAttribute("name", "update");
            UpdatedAttribute("type", "update");
            UpdatedAttribute("uuid", "update");
        }

    }

}
