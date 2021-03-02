using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ApplicationFramesScript : MonoBehaviour
{
    [System.Serializable]
    public class Frame
    {
        public string name;
        public Transform transform;
    }

    public List<Frame> frames = new List<Frame>();

    public Transform GetFrame(string frameName)
    {
        Transform retVal = null;

        foreach (var frame in frames)
        {
            if (frame.name == frameName)
            {
                retVal = frame.transform;
            }
        }

        return retVal;
    }
}
