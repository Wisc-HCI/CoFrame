using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosSharp.RosBridgeClient;

public class RobotMarkerJointStateWriterScript : MonoBehaviour
{
    [System.Serializable]
    public class Joint
    {
        public string name;
        public JointStateWriter writer;
        public List<Renderer> renderers;
    }

    public List<Joint> joints;

    protected bool _visibility = true;

    public void SetJointStates(float[] positions)
    {
        for (int i = 0; (i < positions.Length) && (i < joints.Count); i++)
        {
            joints[i].writer.Write(positions[i]); 
        }
    }

    public void SetJointState(string name, float position)
    {
        for (int i = 0; i < joints.Count; i++)
        {
            if (joints[i].name == name)
            {
                joints[i].writer.Write(position);
            }
        }
    }

    public void SetOpacities(float[] opacities)
    {
        for (int i = 0; (i < opacities.Length) && (i < joints.Count); i++)
        {
            for (int j = 0; j < joints[i].renderers.Count; j++)
            {
                var c = joints[i].renderers[j].material.color;
                c.a = opacities[i];
                joints[i].renderers[j].material.color = c;
            }
        }
    }

    public void SetOpacity(string name, float opacity)
    {
        for (int i = 0; i < joints.Count; i++)
        {
            if (joints[i].name == name)
            {
                for (int j = 0; j < joints[i].renderers.Count; j++)
                {
                    var c = joints[i].renderers[j].material.color;
                    c.a = opacity;
                    joints[i].renderers[j].material.color = c;
                }
            }
        }
    }

    public bool visibility
    {
        get
        {
            return _visibility;
        }

        set
        {
            if (_visibility != value)
            {
                _visibility = value;

                for (int i = 0; i < joints.Count; i++)
                {
                    for (int j = 0; j < joints[i].renderers.Count; j++)
                    {
                        joints[i].renderers[j].enabled = _visibility;
                    }
                }
            }
        }
    }
}
