using System;
using System.Collections;
using System.Collections.Generic;

namespace Cobots
{
    public class AttributeTraceProcessor
    {
        private Dictionary<string,List<Action<Dictionary<string,string>>>> subscribers;

        public AttributeTraceProcessor()
        {
            subscribers = new Dictionary<string, List<Action<Dictionary<string, string>>>>();
        }

        public void SubscribeToType(string type, Action<Dictionary<string, string>> callback)
        {
            if (!subscribers.ContainsKey(type))
            {
                subscribers.Add(type, new List<Action<Dictionary<string, string>>>());
            }

            subscribers[type].Add(callback);
        }

        public bool UnsubscribeToType(string type, Action<Dictionary<string, string>> callback)
        {
            if (!subscribers.ContainsKey(type))
            {
                return false;
            }

            if (!subscribers[type].Contains(callback))
            {
                return false;
            }

            return subscribers[type].Remove(callback);
        }

        public void ProcessTrace(List<Dictionary<string, string>> attributeTrace)
        {
            foreach (var entry in attributeTrace)
            {
                if (subscribers.ContainsKey(entry["type"]))
                {
                    foreach (var cb in subscribers[entry["type"]])
                    {
                        cb(entry);
                    }
                }
            }
        }
    }
}
