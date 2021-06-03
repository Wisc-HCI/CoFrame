using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RobotCollisionMeshScript : CollisionMeshScript
{
    public List<GameObject> collisionRenderers = new List<GameObject>();

    private void Awake()
    {
        foreach (var c in collisionRenderers)
        {
            renderers.AddRange(c.GetComponentsInChildren<Renderer>());
        }

        foreach (var r in renderers)
        {
            r.enabled = visible;

            switch (colorState)
            {
                case ColorState.Good:
                    r.material.color = GOOD_COLOR_STATE_MATERIAL.color;
                    break;
                case ColorState.Warn:
                    r.material.color = WARN_COLOR_STATE_MATERIAL.color;
                    break;
                case ColorState.Error:
                    r.material.color = ERROR_COLOR_STATE_MATERIAL.color;
                    break;
            }
        }
    }

    public override bool visible
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

                foreach (var r in renderers)
                {
                    r.enabled = _visible;
                }
            }
        }
    }
}
