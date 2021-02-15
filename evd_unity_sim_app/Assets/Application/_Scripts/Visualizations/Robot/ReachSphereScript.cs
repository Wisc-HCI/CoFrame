using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ReachSphereScript : MonoBehaviour
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
    private bool _expand = false;
    [SerializeField]
    private ColorState _colorState = ColorState.Good;

    private Animator animator;
    private Renderer sphereRenderer;
    private EvD.Environment.ReachSphere _model;
    private string _uuid;


    private void Start()
    {
        animator = GetComponent<Animator>();

        if (expand)
        {
            animator.ResetTrigger("collapse");
            animator.SetTrigger("expand");
        }
        else
        {
            animator.ResetTrigger("expand");
            animator.SetTrigger("collapse");
        }

        sphereRenderer = GetComponentInChildren<Renderer>();
        switch (colorState)
        {
            case ColorState.Good:
                sphereRenderer.material.color = GOOD_COLOR_STATE_MATERIAL.color;
                break;
            case ColorState.Warn:
                sphereRenderer.material.color = WARN_COLOR_STATE_MATERIAL.color;
                break;
            case ColorState.Error:
                sphereRenderer.material.color = ERROR_COLOR_STATE_MATERIAL.color;
                break;
        }
    }

    public string uuid
    {
        get
        {
            return _uuid;
        }

        private set
        {
            if (_uuid != value)
            {
                _uuid = value;
            }
        }
    }

    public EvD.Environment.ReachSphere model
    {
        get
        {
            return _model;
        }

        set
        {
            if (value != null)
            {
                if (_model != value) 
                {
                    _model = value;
                    uuid = _model.uuid;
                    transform.position = _model.offset.ToUnity();
                    var dim = 2 * _model.radius;
                    transform.localScale = dim * new Vector3(1,1,1);
                }
            }
            else
            {
                _model = null;
                uuid = null;
                transform.position = new Vector3(0,0,0);
                expand = false;
                transform.localScale = new Vector3(1,1,1);
            }
        }
    }

    public bool expand
    {
        get
        {
            return _expand;
        }

        set
        {
            if (_expand != value)
            {
                _expand = value;

                if (_expand)
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
                        sphereRenderer.material.color = GOOD_COLOR_STATE_MATERIAL.color;
                        break;
                    case ColorState.Warn:
                        sphereRenderer.material.color = WARN_COLOR_STATE_MATERIAL.color;
                        break;
                    case ColorState.Error:
                        sphereRenderer.material.color = ERROR_COLOR_STATE_MATERIAL.color;
                        break;
                }
            }
        }
    }
}
