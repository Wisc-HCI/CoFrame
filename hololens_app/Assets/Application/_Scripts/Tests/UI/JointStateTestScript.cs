using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class JointStateTestScript : MonoBehaviour
{
    public JointStateValuesScript jointUI;
    public RobotMarkerJointStateWriterScript robotMarker;

    private void Update()
    {
        float[] fv = jointUI.values;
        double[] dv = new double[fv.Length];
        for (int i = 0; i < dv.Length; i++)
        {
            dv[i] = fv[i];
        }

        robotMarker.SetJointStates(dv);
    }
}
