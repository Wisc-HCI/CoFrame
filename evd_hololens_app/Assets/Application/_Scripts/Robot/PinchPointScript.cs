using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PinchPointScript : MonoBehaviour
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
    private ColorState _colorState = ColorState.Good;

    public Renderer meshRenderer;

    public Animator animator;

    private bool _visible = false;

    private void Awake()
    {
        meshRenderer = GetComponent<Renderer>();
        animator = GetComponent<Animator>();

        meshRenderer.enabled = true;

        switch (_colorState)
        {
            case ColorState.Good:
                meshRenderer.material.color = GOOD_COLOR_STATE_MATERIAL.color;
                break;
            case ColorState.Warn:
                meshRenderer.material.color = WARN_COLOR_STATE_MATERIAL.color;
                break;
            case ColorState.Error:
                meshRenderer.material.color = ERROR_COLOR_STATE_MATERIAL.color;
                break;
        }

        if (visible)
        {
            animator.ResetTrigger("collapse");
            animator.SetTrigger("expand");
        }
        else
        {
            animator.ResetTrigger("expand");
            animator.SetTrigger("collapse");
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
                switch (_colorState)
                {
                    case ColorState.Good:
                        meshRenderer.material.color = GOOD_COLOR_STATE_MATERIAL.color;
                        break;
                    case ColorState.Warn:
                        meshRenderer.material.color = WARN_COLOR_STATE_MATERIAL.color;
                        break;
                    case ColorState.Error:
                        meshRenderer.material.color = ERROR_COLOR_STATE_MATERIAL.color;
                        break;
                }
            }
        }
    }

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
                //meshRenderer.enabled = _visible;

                if (_visible)
                {
                    animator.ResetTrigger("collapse");
                    animator.SetTrigger("expand");
                }
                else
                {
                    animator.ResetTrigger("expand");
                    animator.SetTrigger("collapse");
                }
            }
        }
    }
}
