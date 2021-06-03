using System.Collections;
using System.Collections.Generic;

namespace EvD
{
    public class Program : Task
    {
        /*
         * Attributes
         */

        private Cache _cache = new Cache();
        public System.Action<List<Dictionary<string, string>>> changesCallback { get; set; } = null;

        /*
         * Constructors
         */

        public Program(List<Primitive> primitives = null, System.Action<List<Dictionary<string, string>>> changesCallback = null,
                       string type = "", string name = "", string uuid = null, Node parent = null, bool appendType = true,
                       Context context = null)
        : base(primitives, appendType ? "program." + type : type, name, uuid, parent, appendType, context)
        {
            this.changesCallback = changesCallback;
        }

        public static Program FromDict(Dictionary<string, object> dct, System.Action<List<Dictionary<string, string>>> changesCallback = null)
        {
            var primitives = new List<Primitive>();
            foreach (var p in (List<Dictionary<string, object>>)dct["primitives"])
            {
                primitives.Add(Primitive.FromDict(p));
            }

            return new Program(
                primitives: primitives,
                changesCallback: changesCallback,
                context: Context.FromDict((Dictionary<string, object>)dct["context"]),
                type: (string)dct["type"],
                name: (string)dct["name"],
                uuid: (string)dct["uuid"],
                appendType: false
            );
        }

        /*
         * Accessors and Modifiers
         */
        
        public override Cache cache
        {
            get
            {
                return _cache;
            }
        }

        /*
         * Utility Methods
         */

        public override void ChildChangedEvent(List<Dictionary<string, string>> attributeTrace)
        {
            attributeTrace.Add(ChildChangedEventMsg(null, "callback"));
            changesCallback?.Invoke(attributeTrace);
        }

        public override void UpdatedAttribute(string attribute, string verb, string childUuid = null)
        {
            var evnt = new List<Dictionary<string, string>>
                    {
                        ChildChangedEventMsg(attribute, verb, childUuid)
                    };
            changesCallback?.Invoke(evnt);
        }
    }
}
