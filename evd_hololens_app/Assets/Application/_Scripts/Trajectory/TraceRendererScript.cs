using System.Collections.Generic;
using UnityEngine;

public class TraceRendererScript : MonoBehaviour
{
    public Color END_EFFECTOR_COLOR;
    public Color JOINT_COLOR;
    public Color TOOL_COLOR;
    public Color COMPONENT_COLOR;

    public enum DrawMode
    {
        Graded,
        ColorCoded,
        Inactive
    }

    [System.Serializable]
    public class PathEntry
    {
        public List<string> names = new List<string>();
        public DrawMode mode = DrawMode.Inactive;
        public Color colorCode = new Color();
        public Dictionary<string, List<EvD.TraceDataPoint>> data = new Dictionary<string, List<EvD.TraceDataPoint>>();
        public Dictionary<string, PathRenderScript> scripts = new Dictionary<string, PathRenderScript>();
        public bool visible = false;

        public PathEntry(Color color)
        {
            colorCode = color;
        }
    }

    public GameObject PathPrefab;

    private System.Action<string, Vector3, Quaternion, int> _colliderClickCallback = null;

    private Dictionary<string, PathEntry> pathEntries = new Dictionary<string, PathEntry>();

    private void Awake()
    {
        pathEntries.Add("endEffector", new PathEntry(END_EFFECTOR_COLOR));
        pathEntries.Add("joint", new PathEntry(JOINT_COLOR));
        pathEntries.Add("tool", new PathEntry(TOOL_COLOR));
        pathEntries.Add("component", new PathEntry(COMPONENT_COLOR));
    }

    private bool _visible = false;
    public bool visible
    {
        private get
        {
            return _visible;
        }

        set
        {
            if (_visible != value)
            {
                _visible = value;
                

                foreach (KeyValuePair<string, PathEntry> entry in pathEntries)
                {
                    entry.Value.visible = _visible;
                    foreach(KeyValuePair<string, PathRenderScript> script in entry.Value.scripts)
                    {
                        script.Value.gameObject.SetActive(_visible);
                    }
                }
            }
        }
    }

    public System.Action<string, Vector3, Quaternion, int> colliderClickCallback
    {
        private get
        {
            return _colliderClickCallback;
        }

        set
        {
            if (_colliderClickCallback != value)
            {
                _colliderClickCallback = value;
                foreach (var p in pathEntries)
                {
                    foreach (var s in p.Value.scripts)
                    {
                        s.Value.colliderClickCallback = _colliderClickCallback;
                    }
                }
            }
        }
    }

    public void SetEndEffectorPathVisibility(bool val)
    {
        SetVisibleSubset("endEffector", val);
    }

    public void SetJointPathVisibility(bool val)
    {
        SetVisibleSubset("joint", val);
    }

    public void SetToolPathVisibility(bool val)
    {
        SetVisibleSubset("tool", val);
    }

    public void SetComponentPathVisibility(bool val)
    {
        SetVisibleSubset("component", val);
    }

    public bool GetEndEffectorPathVisibility()
    {
        return pathEntries["endEffector"].visible;
    }

    public bool GetJointPathVisibility()
    {
        return pathEntries["joint"].visible;
    }

    public bool GetToolPathVisibility()
    {
        return pathEntries["tool"].visible;
    }

    public bool GetComponentPathVisibility()
    {
        return pathEntries["component"].visible;
    }

    public void SetEndEffectorPathDrawMode(DrawMode mode)
    {
        if (mode != pathEntries["endEffector"].mode)
        {
            pathEntries["endEffector"].mode = mode;
            DrawSubset("endEffector");
        }
    }

    public void SetJointPathDrawMode(DrawMode mode)
    {
        if (mode != pathEntries["joint"].mode)
        {
            pathEntries["joint"].mode = mode;
            DrawSubset("joint");
        }
    }

    public void SetToolPathDrawMode(DrawMode mode)
    {
        if (mode != pathEntries["tool"].mode)
        {
            pathEntries["tool"].mode = mode;
            DrawSubset("tool");
        }
    }

    public void SetComponentPathDrawMode(DrawMode mode)
    {
        if (mode != pathEntries["component"].mode)
        {
            pathEntries["component"].mode = mode;
            DrawSubset("component");
        }
    }

    public DrawMode GetEndEffectorPathDrawMode()
    {
        return pathEntries["endEffector"].mode;
    }

    public DrawMode GetJointPathDrawMode()
    {
        return pathEntries["joint"].mode;
    }

    public DrawMode GetToolPathDrawMode()
    {
        return pathEntries["tool"].mode;
    }

    public DrawMode GetComponentPathDrawMode()
    {
        return pathEntries["component"].mode;
    }

    public void OnUpdate(EvD.Trace t)
    {
        // Generate end-effector path
        var l = new List<string>();
        l.Add(t.endEffectorPath);
        pathEntries["endEffector"].names = l;
        UpdateSubset("endEffector", t);

        // Generate joint-paths
        pathEntries["joint"].names = t.jointPaths;
        UpdateSubset("joint", t);

        // Generate tool-paths
        pathEntries["tool"].names = t.toolPaths;
        UpdateSubset("tool", t);

        // Generate component-paths
        pathEntries["component"].names = t.componentPaths;
        UpdateSubset("component", t);
    }

    public void OnDelete()
    {
        foreach (var entry in pathEntries)
        {
            entry.Value.names.Clear();
            foreach (var script in entry.Value.scripts)
            {
                Destroy(script.Value.gameObject);
            }
            entry.Value.scripts.Clear();
        }
    }

    private void DrawSubset(string subset)
    {
        foreach (var pn in pathEntries[subset].names)
        {
            pathEntries[subset].scripts[pn].Draw(pathEntries[subset].data[pn], (PathRenderScript.DrawMode)(pathEntries[subset].mode), pathEntries[subset].colorCode);
        }
    }

    private void SetVisibleSubset(string subset, bool value)
    {
        if (pathEntries[subset].visible != value)
        {
            pathEntries[subset].visible = value;
            foreach (var pn in pathEntries[subset].names)
            {
                pathEntries[subset].scripts[pn].gameObject.SetActive(value);
            }
        }
    }
    
    private void UpdateSubset(string subset, EvD.Trace t)
    {
        pathEntries[subset].data.Clear();
        foreach (var n in pathEntries[subset].names)
        {
            pathEntries[subset].data.Add(n, t.data[n]);
        }


        foreach (var name in pathEntries[subset].names)
        {
            if (!pathEntries[subset].scripts.ContainsKey(name))
            {
                var g = Instantiate(PathPrefab, transform);
                var s = g.GetComponent<PathRenderScript>();
                pathEntries[subset].scripts.Add(name, s);
                s.gameObject.SetActive(pathEntries[subset].visible);
            }

            pathEntries[subset].scripts[name].Draw(pathEntries[subset].data[name], (PathRenderScript.DrawMode)(pathEntries[subset].mode), pathEntries[subset].colorCode);
        }
    }
}

