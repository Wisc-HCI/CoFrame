using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MarkerPoseEditorScript : MonoBehaviour
{
    public VectorDisplayScript position;
    public VectorDisplayScript orientation;
    public System.Action closeCallback = null;
    public EvD.Waypoint waypoint = null;

    private void Update()
    {
        if (waypoint != null)
        {
            position.data = waypoint.position.ToUnity();
            orientation.data = waypoint.orientation.ToUnity().eulerAngles;
        }
    }

    public void OnRandomButtonPress()
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

        waypoint.position = EvD.Position.FromUnity(pos);
        waypoint.orientation = EvD.Orientation.FromUnity(Quaternion.Euler(rot));
    }

    public void OnCloseButtonClick()
    {
        closeCallback?.Invoke();
    }
}
