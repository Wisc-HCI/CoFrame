using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RenderPointScript : MonoBehaviour
{
    public float WIDTH = 0.0075f;
    public float OPACITY = 0.75f;
    public Color GRADE_ZERO_COLOR = new Color(255 / 255,242 / 255, 117 / 255);
    public Color GRADE_ONE_COLOR = new Color(255 / 255, 60 / 255, 56 / 255);
    public Color INACTIVE_COLOR = new Color(185 / 255, 185 / 255, 185 / 255);

    private LineRenderer lineRenderer;
    private MeshRenderer meshRenderer;

    public void Instantiate()
    {
        lineRenderer = GetComponent<LineRenderer>();
        lineRenderer.startWidth = WIDTH;
        lineRenderer.endWidth = WIDTH;
        lineRenderer.material = new Material(Shader.Find("Sprites/Default"));

        meshRenderer = GetComponentInChildren<MeshRenderer>();
    }

    public void VisiblePoint(bool value)
    {
        meshRenderer.enabled = value;
    }

    public void VisibleLine(bool value)
    {
        lineRenderer.enabled = value;
    }

    public void DrawInactive(Transform other)
    {
        Draw(other);

        var gradient = new Gradient();
        gradient.SetKeys(
            new GradientColorKey[] { new GradientColorKey(INACTIVE_COLOR, 0.0f), new GradientColorKey(INACTIVE_COLOR, 1.0f) },
            new GradientAlphaKey[] { new GradientAlphaKey(OPACITY, 0.0f), new GradientAlphaKey(OPACITY, 1.0f) }
        );
        lineRenderer.colorGradient = gradient;

        var c = INACTIVE_COLOR;
        c.a = OPACITY;
        meshRenderer.material.color = c;
    }

    public void DrawGrade(Transform other, float otherGrade, float selfGrade)
    {
        Draw(other);

        var endColor = ColorFromGrade(otherGrade);

        var gradient = new Gradient();
        gradient.SetKeys(
            new GradientColorKey[] { new GradientColorKey(ColorFromGrade(selfGrade),0.0f), new GradientColorKey(endColor, 1.0f) }, 
            new GradientAlphaKey[] { new GradientAlphaKey(OPACITY, 0.0f), new GradientAlphaKey(OPACITY, 1.0f) }
        );
        lineRenderer.colorGradient = gradient;

        var c = endColor;
        c.a = OPACITY;
        meshRenderer.material.color = c;
    }

    public void DrawColorCoded(Transform other, Color color)
    {
        Draw(other);

        var gradient = new Gradient();
        gradient.SetKeys(
            new GradientColorKey[] { new GradientColorKey(color, 0.0f), new GradientColorKey(color, 1.0f) },
            new GradientAlphaKey[] { new GradientAlphaKey(OPACITY, 0.0f), new GradientAlphaKey(OPACITY, 1.0f) }
        );
        lineRenderer.colorGradient = gradient;

        var c = color;
        c.a = OPACITY;
        meshRenderer.material.color = c;
    }

    private void Draw(Transform other)
    {
        var points = new Vector3[2];
        points[0] = other.position;
        points[1] = transform.position;

        lineRenderer.positionCount = 2;
        lineRenderer.SetPositions(points);
    }

    private Color ColorFromGrade(float grade)
    {
        return Color.Lerp(GRADE_ZERO_COLOR, GRADE_ONE_COLOR, grade);
    }
}
