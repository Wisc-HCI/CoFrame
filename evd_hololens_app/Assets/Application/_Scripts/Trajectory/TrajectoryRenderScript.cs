using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using EvD;

public class TrajectoryRenderScript : MonoBehaviour
{
    public GameObject LineColliderPrefab;

    public enum DrawMode
    {
        Active,
        Inactive
    }

    private System.Action<string, Vector3, Quaternion, int> _colliderClickCallback = null;

    private TrajectoryPointScript startPoint = null;
    private TrajectoryPointScript endPoint = null;
    private List<TrajectoryPointScript> waypoints = null;
    private List<TrajectoryPointScript> allPoints = new List<TrajectoryPointScript>();
    private List<LineColliderScript> colliders = new List<LineColliderScript>();

    [SerializeField]
    private bool _visible = true;
    public bool visible
    {
        get
        {
            return _visible;
        }

        set
        {
            if (_visible != value)
            {
                _visible = value;

                foreach (var p in allPoints)
                {
                    p.VisibleLine(_visible);
                }
                foreach (var c in colliders)
                {
                    c.gameObject.SetActive(_visible);
                }
            }
        }
    }

    public string uuid { get; set; }

    private DrawMode _mode = DrawMode.Active;
    public DrawMode mode
    {
        get
        {
            return _mode;
        }

        set
        {
            if (_mode != value)
            {
                _mode = value;
                switch (mode)
                {
                    case DrawMode.Active:
                        for (int i = 0; i < allPoints.Count; i++)
                        {
                            if (i > 0)
                            {
                                allPoints[i].DrawActive(allPoints[i - 1].transform);
                            }
                        }
                        break;

                    case DrawMode.Inactive:
                        for (int i = 0; i < allPoints.Count; i++)
                        {
                            if (i > 0)
                            {
                                allPoints[i].DrawInactive(allPoints[i - 1].transform);
                            }
                        }
                        break;
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
                foreach (var c in colliders)
                {
                    c.callback = _colliderClickCallback;
                }
            }
        }
    }

    public void OnCreate(TrajectoryPointScript start, TrajectoryPointScript end, List<TrajectoryPointScript> waypoints)
    {
        startPoint = start;
        endPoint = end;
        this.waypoints = waypoints;

        allPoints.Add(startPoint);
        allPoints.AddRange(waypoints);
        allPoints.Add(endPoint);

        FullRender();
    }

    public void OnAddWaypoint(TrajectoryPointScript waypoint, int idx)
    {
        LocalClear(idx + 1, false);

        allPoints.Clear();

        waypoints.Insert(idx, waypoint);

        allPoints.Add(startPoint);
        allPoints.AddRange(waypoints);
        allPoints.Add(endPoint);

        LocalRender(idx + 1);
    }

    public void OnReorderWaypoints()
    {
        FullClear();
        FullRender();
    }

    public void OnDeleteWaypoint(string uuid)
    {
        int idx = FindIndexById(uuid);

        LocalClear(idx);

        allPoints.RemoveAt(idx);
        waypoints.RemoveAt(idx - 1);

        LocalRender(idx - 1, false);
    }

    public void OnManipulatedWaypoint(string uuid)
    {
        int idx = FindIndexById(uuid);
        if (idx >= 0)
        {
            LocalClear(idx);
            LocalRender(idx);
        }
    }

    public void OnDelete()
    {
        FullClear();
        allPoints.Clear();
        waypoints = null;
        startPoint = null;
        endPoint = null;
    }

    private int FindIndexById(string uuid)
    {
        for (int i = 0; i < allPoints.Count; i++)
        {
            if (allPoints[i].uuid == uuid)
            {
                return i;
            }
        }
        return -1;
    }

    private void FullRender()
    {
        for (int i = 0; i < allPoints.Count; i++)
        {
            if (i > 0)
            {
                switch (mode)
                {
                    case DrawMode.Active:
                        allPoints[i].DrawActive(allPoints[i - 1].transform);
                        break;
                    case DrawMode.Inactive:
                        allPoints[i].DrawInactive(allPoints[i - 1].transform);
                        break;
                }

                colliders.Add(CreateCollider(i));
            }
        }
    }

    private void FullClear()
    {
        foreach (var p in allPoints)
        {
            p.Clear();
        }

        foreach (var c in colliders)
        {
            Destroy(c.gameObject);
        }
        colliders.Clear();
    }

    private void LocalRender(int idx, bool before = true)
    {
        // draw at
        if (idx > 0 && before)
        {
            switch (mode)
            {
                case DrawMode.Active:
                    allPoints[idx].DrawActive(allPoints[idx - 1].transform);
                    break;
                case DrawMode.Inactive:
                    allPoints[idx].DrawInactive(allPoints[idx - 1].transform);
                    break;
            }
            colliders.Insert(idx - 1, CreateCollider(idx));
        }

        // draw after
        if (idx < allPoints.Count-1)
        {
            switch (mode)
            {
                case DrawMode.Active:
                    allPoints[idx + 1].DrawActive(allPoints[idx].transform);
                    break;
                case DrawMode.Inactive:
                    allPoints[idx + 1].DrawInactive(allPoints[idx].transform);
                    break;
            }
            colliders.Insert(idx, CreateCollider(idx + 1));
        }

    }

    private void LocalClear(int idx, bool after = true)
    {
        // clear at
        if (idx > 0)
        {
            allPoints[idx].Clear();
            Destroy(colliders[idx - 1].gameObject);
            colliders.RemoveAt(idx - 1);
        }

        // clear after
        if (idx < allPoints.Count - 1 && after)
        {
            allPoints[idx + 1].Clear();
            
            if (idx > 0)
            {
                Destroy(colliders[idx - 1].gameObject);
                colliders.RemoveAt(idx - 1);
            }
            else
            {
                Destroy(colliders[idx].gameObject);
                colliders.RemoveAt(idx);
            }
        }
    }

    private LineColliderScript CreateCollider(int i)
    {
        var lc = Instantiate(LineColliderPrefab, transform, false);
        var translate = Vector3.Lerp(allPoints[i - 1].transform.localPosition, allPoints[i].transform.localPosition, 0.5f);
        lc.transform.localPosition = translate;
        lc.transform.LookAt(allPoints[i - 1].transform.position);

        var cc = lc.GetComponent<CapsuleCollider>();
        cc.radius = 2 * allPoints[i].WIDTH;
        cc.height = Vector3.Distance(allPoints[i - 1].transform.position, allPoints[i].transform.position) + allPoints[i].WIDTH;
        cc.direction = 2;

        var ls = lc.GetComponent<LineColliderScript>();
        ls.id = uuid;
        ls.index = i - 1;
        ls.prevPoint = allPoints[i - 1].transform;
        ls.currPoint = allPoints[i].transform;
        ls.callback = colliderClickCallback;

        return ls;
    }
}
