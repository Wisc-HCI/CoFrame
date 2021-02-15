using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SpawnManagerScript : MonoBehaviour
{
    [System.Serializable]
    public class SpawnEntry
    {
        public GameObject target;
        public string name;
    }

    public SpawnEntry[] spawnEntries;

    public Transform GetSpawnTargetTransform(string name)
    {

        foreach (var s in spawnEntries)
        {
            if (s.name == name)
            {
                return s.target.transform;
            }
        }

        return null;
    }
}
