using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class JointStateEditorScript : MonoBehaviour
{
    public JointStateValuesScript jointUI;
    public System.Action closeCallback = null;

    private Cobots.Waypoint _waypoint = null;
    private double[] _prevData = null;
    private float[] _prevJointUIData = null;

    private void Update()
    {
        if (waypoint != null)
        {
            float[] fv = jointUI.values;
            double[] dv = new double[fv.Length];
            for (int i = 0; i < dv.Length; i++)
            {
                dv[i] = fv[i];
            }

            waypoint.joints = dv;
        }
    }

    public Cobots.Waypoint waypoint
    {
        get
        {
            return _waypoint;
        }

        set
        {
            if (_waypoint != value)
            {
                _waypoint = null; // disable update loop

                var temp = value;

                // copy initial values if already set
                if (temp != null && temp.joints != null)
                {
                    _prevData = temp.joints;

                    double[] dv = temp.joints;
                    float[] fv = new float[temp.joints.Length];
                    for (int i = 0; i < dv.Length; i++)
                    {
                        fv[i] = (float)(dv[i]);
                    }

                    _prevJointUIData = jointUI.values;
                    jointUI.values = fv;
                }
                else
                {
                    _prevData = null;
                }
                
                _waypoint = temp;
            }
        }
    }

    public void OnCancelButtonClick()
    {
        waypoint.joints = _prevData;

        if (_prevJointUIData != null)
        {
            jointUI.values = _prevJointUIData;
        }
        else
        {
            jointUI.values = jointUI.defaultValues;
        }

        closeCallback?.Invoke();
    }

    public void OnSaveButtonClick()
    {
        _prevJointUIData = jointUI.values;
        closeCallback?.Invoke();
    }
}
