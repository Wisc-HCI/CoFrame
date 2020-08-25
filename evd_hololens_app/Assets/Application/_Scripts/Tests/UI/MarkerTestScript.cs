using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class MarkerTestScript : MonoBehaviour
{
    public Cobots.AbstractMarker marker;

    public VectorDisplayScript position;
    public VectorDisplayScript orientation;

    public Toggle transparentToggle;
    public Toggle particlesToggle;

    public InputField nameField;
    public Toggle nameVisibleToggle;
    public Toggle nameInteractableToggle;

    private void Start()
    {
        OnTransparentToggleChanged();
        OnParticlesToggleChanged();
        OnButtonPress();
        OnNameFieldEditComplete();
        OnNameVisibleToggleChanged();
        OnNameInteractableToggleChanged();
    }

    private void Update()
    {
        position.data = marker.Position;
        orientation.data = marker.Orientation.eulerAngles;
    }

    public void OnButtonPress()
    {
        // Position
        Vector3 pos = new Vector3(
            Random.Range(-1.0f, 1.0f),
            Random.Range(-1.0f, 1.0f),
            Random.Range(-0.0f, 1.0f)
        );
        
        // Orientation
        Vector3 rot = new Vector3(
            Random.Range(-180, 180),
            Random.Range(-180, 180),
            Random.Range(-180, 180)
        );

        // Set Marker
        marker.Position = pos;
        marker.Orientation = Quaternion.Euler(rot);
    }

    public void OnTransparentToggleChanged()
    {
        if (transparentToggle.isOn)
        {
            marker.MakeTranslucent();
        }
        else
        {
            marker.MakeOpaque();
        }
    }

    public void OnParticlesToggleChanged()
    {
        if (particlesToggle.isOn)
        {
            marker.StartParticles();
        }
        else
        {
            marker.StopParticles();
        }
    }

    public void OnNameFieldEditComplete()
    {
        marker.name = nameField.text;
    }

    public void OnNameVisibleToggleChanged()
    {
        marker.isNameVisible = nameVisibleToggle.isOn;
    }

    public void OnNameInteractableToggleChanged()
    {
        marker.allowNameInteraction = nameInteractableToggle.isOn;
    }

    private void OnNameTagUpdateCallback(Cobots.AbstractMarker m, string name)
    {
        nameField.text = name;
    }
}
