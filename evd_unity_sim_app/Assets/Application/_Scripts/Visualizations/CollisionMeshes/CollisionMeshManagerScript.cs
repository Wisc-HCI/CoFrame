using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CollisionMeshManagerScript : MonoBehaviour
{
    [System.Serializable]
    public class CollisionMeshEntry
    {
        public string name;
        public CollisionMeshScript script;
    }

    public List<CollisionMeshEntry> collisionMeshes = new List<CollisionMeshEntry>();

    public void SetVisible(string name, bool val)
    {
        foreach (var c in collisionMeshes)
        {
            if (name == c.name)
            {
                c.script.visible = val;
            }
        }
    }

    public bool GetVisible(string name)
    {
        bool val = false;
        foreach (var c in collisionMeshes)
        {
            if (name == c.name)
            {
                val = c.script.visible;
            }
        }
        return val;
    }

    public void OnCollisionMeshUpdate(EvD.Environment.CollisionMesh cState)
    {
        foreach (var c in collisionMeshes)
        {
            if (cState.name == c.name)
            {
                /*
                if (cState.state == "good")
                {
                    c.script.colorState = CollisionMeshScript.ColorState.Good;
                }
                else if (cState.state == "warn")
                {
                    c.script.colorState = CollisionMeshScript.ColorState.Warn;
                }
                else if (cState.state == "error")
                {
                    c.script.colorState = CollisionMeshScript.ColorState.Error;
                }
                */
            }
        }
    }
}
