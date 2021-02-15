using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PathRenderScript : MonoBehaviour
{
    public bool ADD_COLLIDER = true;

    public enum DrawMode
    {
        Graded,
        ColorCoded,
        Inactive
    }

    public GameObject RenderPointPrefab;
    public GameObject LineColliderPrefab;

    private System.Action<string, Vector3, Quaternion, int> _colliderClickCallback = null;

    public new string name;

    [SerializeField]
    private bool _visible = true;
    [SerializeField]
    private bool _displayPoints = true;
    [SerializeField]
    private bool _displayLines = true;
    [SerializeField]
    public DrawMode mode { get; private set; }

    private List<RenderPointScript> renderPoints = new List<RenderPointScript>();
    private List<LineColliderScript> lineColliders = new List<LineColliderScript>();

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

                foreach (var p in renderPoints)
                {
                    p.gameObject.SetActive(_visible);
                }

                foreach (var c in lineColliders)
                {
                    c.gameObject.SetActive(_visible);
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
                foreach (var c in lineColliders)
                {
                    c.callback = _colliderClickCallback;
                }
            }
        }
    }

    public void Draw(List<EvD.Data.TraceDataPoint> data, DrawMode mode, Color colorCode = new Color())
    {
        this.mode = mode;

        // Make sure path is clear
        Clear();

        // Create render points and line colliders
        for (int i = 0; i < data.Count; i++)
        {
            var p = Instantiate(RenderPointPrefab, transform, false);
            p.transform.localPosition = data[i].position.ToUnity();
            p.transform.localRotation = data[i].orientation.ToUnity();
            p.SetActive(visible);

            var rs = p.GetComponent<RenderPointScript>();
            rs.Instantiate();
            rs.VisiblePoint(_displayPoints);
            rs.VisibleLine(_displayLines);

            renderPoints.Add(rs);

            if (i > 0)
            {
                switch (mode)
                {
                    case DrawMode.ColorCoded:
                        rs.DrawColorCoded(renderPoints[i - 1].transform, colorCode);
                        break;
                    case DrawMode.Graded:
                        rs.DrawGrade(renderPoints[i - 1].transform, data[i - 1].grade, data[i].grade);
                        break;
                    case DrawMode.Inactive:
                        rs.DrawInactive(renderPoints[i - 1].transform);
                        break;
                }

                if (ADD_COLLIDER)
                {
                    lineColliders.Add(CreateCollider(i,rs));
                }
            }
        }
    }

    public void Clear()
    {
        foreach (var p in renderPoints)
        {
            Destroy(p.gameObject);
        }
        renderPoints.Clear();

        foreach (var c in lineColliders)
        {
            Destroy(c.gameObject);
        }
        lineColliders.Clear();
    }

    private LineColliderScript CreateCollider(int i, RenderPointScript rs)
    {
        var lc = Instantiate(LineColliderPrefab, transform, false);
        var translate = Vector3.Lerp(renderPoints[i - 1].transform.localPosition, renderPoints[i].transform.localPosition, 0.5f);
        lc.transform.Translate(translate);
        lc.transform.LookAt(renderPoints[i - 1].transform.position);

        var cc = lc.GetComponent<CapsuleCollider>();
        cc.radius = rs.WIDTH;
        cc.height = Vector3.Distance(renderPoints[i - 1].transform.position, renderPoints[i].transform.position) + rs.WIDTH;
        cc.direction = 2;

        var ls = lc.GetComponent<LineColliderScript>();
        ls.id = name;
        ls.callback = colliderClickCallback;

        return ls;
    }
}
