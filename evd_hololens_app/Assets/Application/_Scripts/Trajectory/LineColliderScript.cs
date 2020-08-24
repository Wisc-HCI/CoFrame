using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class LineColliderScript : MonoBehaviour
{
    public string id;
    public int index;
    public System.Action<string, Vector3, Quaternion, int> callback = null;

    public Transform prevPoint = null;
    public Transform currPoint = null;

    public void OnInteractionClicked()
    {
        Quaternion orientation;
        if (prevPoint != null && currPoint != null)
        {
            orientation = Quaternion.Lerp(prevPoint.localRotation, currPoint.localRotation, 0.5f);
        }
        else
        {
            orientation = Quaternion.identity;
        }

        callback?.Invoke(id, transform.localPosition, orientation, index);
    }
}
