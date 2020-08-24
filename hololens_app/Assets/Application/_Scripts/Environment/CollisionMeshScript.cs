using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CollisionMeshScript : MonoBehaviour
{
    public enum ColorState
    {
        Good,
        Warn,
        Error
    }

    public Material GOOD_COLOR_STATE_MATERIAL;
    public Material WARN_COLOR_STATE_MATERIAL;
    public Material ERROR_COLOR_STATE_MATERIAL;

    [SerializeField]
    protected ColorState _colorState = ColorState.Good;

    [SerializeField]
    protected List<Renderer> renderers = new List<Renderer>();

    [SerializeField]
    protected List<Animator> animators = new List<Animator>();

    [SerializeField]
    protected bool _visible = false;

    private void Awake()
    {
        renderers = new List<Renderer>(GetComponentsInChildren<Renderer>());
        animators = new List<Animator>(GetComponentsInChildren<Animator>());

        foreach(var r in renderers)
        {
            r.enabled = true;

            switch (_colorState)
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

        foreach(var a in animators)
        {
            if (visible)
            {
                a.ResetTrigger("collapse");
                a.SetTrigger("expand");
            }
            else
            {
                a.ResetTrigger("expand");
                a.SetTrigger("collapse");
            }
        }
    }

    public ColorState colorState
    {
        get
        {
            return _colorState;
        }

        set
        {
            if (_colorState != value)
            {
                _colorState = value;
                foreach (var r in renderers)
                {
                    switch (_colorState)
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
        }
    }

    public virtual bool visible
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
                
                /*
                foreach (var r in renderers)
                {
                    r.enabled = _visible;
                }
                */

                foreach (var a in animators)
                {
                    if (_visible)
                    {
                        a.ResetTrigger("collapse");
                        a.SetTrigger("expand");
                    }
                    else
                    {
                        a.ResetTrigger("expand");
                        a.SetTrigger("collapse");
                    }
                }
            }
        }
    }
}
