using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TrajectoryPointScript : MonoBehaviour
{
    public float WIDTH = 0.0075f;
    public float OPACITY = 0.75f;

    public Color ACTIVE_COLOR = new Color(185 / 255, 185 / 255, 185 / 255);
    public Color INACTIVE_COLOR = new Color(185 / 255, 185 / 255, 185 / 255);

    public string uuid { get; set; }

    private LineRenderer lineRenderer;

    public new Transform transform{ get; private set; }

    public virtual void Instantiate(string uuid,  Transform markerTf, bool visibleLine = true)
    {
        this.uuid = uuid;
        this.transform = markerTf;

        lineRenderer = GetComponent<LineRenderer>();
        lineRenderer.startWidth = WIDTH;
        lineRenderer.endWidth = WIDTH;
        lineRenderer.material = new Material(Shader.Find("Sprites/Default"));
        VisibleLine(visibleLine);
    }

    public virtual void VisibleLine(bool value)
    {
        lineRenderer.enabled = value;
    }

    public virtual void DrawInactive(Transform other)
    {
        Draw(other);

        var gradient = new Gradient();
        gradient.SetKeys(
            new GradientColorKey[] { new GradientColorKey(INACTIVE_COLOR, 0.0f), new GradientColorKey(INACTIVE_COLOR, 1.0f) },
            new GradientAlphaKey[] { new GradientAlphaKey(OPACITY, 0.0f), new GradientAlphaKey(OPACITY, 1.0f) }
        );
        lineRenderer.colorGradient = gradient;
    }

    public virtual void DrawActive(Transform other)
    {
        Draw(other);

        var gradient = new Gradient();
        gradient.SetKeys(
            new GradientColorKey[] { new GradientColorKey(ACTIVE_COLOR, 0.0f), new GradientColorKey(ACTIVE_COLOR, 1.0f) },
            new GradientAlphaKey[] { new GradientAlphaKey(OPACITY, 0.0f), new GradientAlphaKey(OPACITY, 1.0f) }
        );
        lineRenderer.colorGradient = gradient;
    }

    protected virtual void Draw(Transform other)
    {
        var points = new Vector3[2];
        points[0] = other.position;
        points[1] = transform.position;

        lineRenderer.positionCount = 2;
        lineRenderer.SetPositions(points);
    }

    public virtual void Clear()
    {
        lineRenderer.positionCount = 0;
        lineRenderer.SetPositions(new Vector3[0]);
    }
}
