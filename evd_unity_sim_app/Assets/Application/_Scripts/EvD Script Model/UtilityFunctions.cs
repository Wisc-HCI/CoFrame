using System.Collections;
using System.Collections.Generic;


namespace EvD
{
    [System.Serializable]
    public class UtilityFunctions
    {

        public static Node NodeParser(Dictionary<string,object> dct) 
        {
            Node node = null;

            // Check if object already in the cache
            try {
                node = Cache.Get((string)dct["uuid"]);
            } catch (System.Exception) {
                node = null;
            }
            
            // Must Create a new object
            //TODO

            return node;
        }

        public static string GetExactType(Node n)
        {
            string[] type = n.type.Split('.');
            string exactType = type[type.Length - 2];
            return exactType;
        }

    }
}