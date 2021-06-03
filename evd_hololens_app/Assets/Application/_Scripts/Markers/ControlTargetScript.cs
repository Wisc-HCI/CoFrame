using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ControlTargetScript : MonoBehaviour
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
    [SerializeField]
    private MarkerMeshTypeScript.Options _markerType = MarkerMeshTypeScript.Options.Location;

    private Renderer meshRenderer;
    private MarkerMeshTypeScript markerTypeScript;

    private void Awake()
    {
        markerTypeScript = GetComponentInChildren<MarkerMeshTypeScript>();
        markerTypeScript.SetMeshType(markerType);

        meshRenderer = GetComponentInChildren<Renderer>();
        switch (colorState)
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

    public MarkerMeshTypeScript.Options markerType
    {
        get
        {
            return _markerType;
        }

        set
        {
            if (_markerType != value)
            {
                _markerType = value;
                markerTypeScript.SetMeshType(_markerType);
            }
        }
    }


    public void OnManipulationStarted()
    {
        // TODO hook into environment node
    }

    public void OnManipulationEnded()
    {
        // TODO hook into environment node
    }
}
