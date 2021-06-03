using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CustomArrowTrajectoryPointScript : TrajectoryPointScript
{
    private CustomArrowRenderer arrowRenderer;

    public float SPEED = 1f;

    public override void Instantiate(string uuid, Transform markerTf, bool visibleLine = true)
    {
        arrowRenderer = GetComponentInChildren<CustomArrowRenderer>();
        base.Instantiate(uuid, markerTf, visibleLine);
    }

    public override void VisibleLine(bool value)
    {
        base.VisibleLine(value);
        arrowRenderer.gameObject.SetActive(value);
    }

    public override void DrawInactive(Transform other)
    {
        arrowRenderer.SetColor(INACTIVE_COLOR);
        arrowRenderer.speed = 0;
        base.DrawInactive(other);
    }

    public override void DrawActive(Transform other)
    {
        arrowRenderer.SetColor(ACTIVE_COLOR);
        arrowRenderer.speed = SPEED;
        base.DrawActive(other);
    }

    protected override void Draw(Transform other)
    {
        base.Draw(other);
        arrowRenderer.Draw = true;
        arrowRenderer.SetPositions(other.position, transform.position);
    }

    public override void Clear()
    {
        base.Clear();
        arrowRenderer.Draw = false;
    }
}
