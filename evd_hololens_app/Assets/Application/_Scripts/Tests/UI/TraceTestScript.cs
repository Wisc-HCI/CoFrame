using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class TraceTestScript : MonoBehaviour
{
    public int NUMBER_OF_TEST_POINTS = 100;

    public TraceRendererScript trace;

    public Toggle[] toggles;
    public Dropdown[] dropdowns;

    private void Start()
    {
        // Set display states
        for (int i=0; i<4; i++)
        {
            switch (i)
            {
                case 0:
                    toggles[i].isOn = trace.GetEndEffectorPathVisibility();
                    dropdowns[i].value = (int)(trace.GetEndEffectorPathDrawMode());
                    break;
                case 1:
                    toggles[i].isOn = trace.GetJointPathVisibility();
                    dropdowns[i].value = (int)(trace.GetJointPathDrawMode());
                    break;
                case 2:
                    toggles[i].isOn = trace.GetToolPathVisibility();
                    dropdowns[i].value = (int)(trace.GetToolPathDrawMode());
                    break;
                case 3:
                    toggles[i].isOn = trace.GetComponentPathVisibility();
                    dropdowns[i].value = (int)(trace.GetComponentPathDrawMode());
                    break;
            }
        }

        // Generate initial data
        OnUpdateButton();
    }

    public void OnVisibilityToggle(int idx)
    {
        switch(idx)
        {
            case 0:
                trace.SetEndEffectorPathVisibility(toggles[idx].isOn);
                break;
            case 1:
                trace.SetJointPathVisibility(toggles[idx].isOn);
                break;
            case 2:
                trace.SetToolPathVisibility(toggles[idx].isOn);
                break;
            case 3:
                trace.SetComponentPathVisibility(toggles[idx].isOn);
                break;
        }
    }

    public void OnModeDropdown(int idx)
    {
        switch (idx)
        {
            case 0:
                trace.SetEndEffectorPathDrawMode((TraceRendererScript.DrawMode)(dropdowns[idx].value));
                break;
            case 1:
                trace.SetJointPathDrawMode((TraceRendererScript.DrawMode)(dropdowns[idx].value));
                break;
            case 2:
                trace.SetToolPathDrawMode((TraceRendererScript.DrawMode)(dropdowns[idx].value));
                break;
            case 3:
                trace.SetComponentPathDrawMode((TraceRendererScript.DrawMode)(dropdowns[idx].value));
                break;
        }
    }

    public void OnUpdateButton()
    {
        // Ranges for Random Generation
        Vector3 minPos = new Vector3(-0.5f, -0.5f, -0.5f);
        Vector3 maxPos = new Vector3(0.5f, 0.5f, 0.5f);
        Vector3 minRot = new Vector3(0.0f,0.0f,0.0f);
        Vector3 maxRot = new Vector3(6.28f, 6.28f, 6.28f);

        // path names
        string eePath = "ee";
        List<string> jointPaths = new List<string>() { "j1", "j2", "j3", "j4", "j5", "j6" };
        List<string> toolPaths = new List<string>() { "t1", "t2" };
        List<string> compPaths = new List<string>() { "c1" };

        List<string> allPaths = new List<string>();
        allPaths.Add(eePath);
        allPaths.AddRange(jointPaths);
        allPaths.AddRange(toolPaths);
        allPaths.AddRange(compPaths);

        // generate random new trace
        Dictionary<string, List<EvD.TraceDataPoint>> data = new Dictionary<string, List<EvD.TraceDataPoint>>();
        foreach (string name in allPaths)
        {
            data[name] = new List<EvD.TraceDataPoint>();
            for (int i=0; i<NUMBER_OF_TEST_POINTS; i++)
            {
                float grade = Random.Range(0.0f, 1.0f);
                var tp = new EvD.TraceDataPoint(EvD.Position.GenerateRandom(minPos, maxPos), EvD.Orientation.GenerateRandom(minRot, maxRot),grade);
                data[name].Add(tp);
            }
        }
        
        // pack and update
        EvD.Trace t = new EvD.Trace(eePath, data, jointPaths, toolPaths, compPaths, 10);
        trace.OnUpdate(t);
    }

    public void OnDeleteButton()
    {
        trace.OnDelete();
    }
}
