using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraControlScript : MonoBehaviour
{
    public GameObject TargetObject;

    public float MaxOffset;

    public float MaxVerticalRotation;
    public float MinVerticalRotation;

    public float MaxZoom;
    public float MinZoom;

    private Vector3 Offset = new Vector3();

    private float VerticalAngle = 0;
    private float HorizontalAngle = 0;

    private float Zoom = 1;

    // Start is called before the first frame update
    void Start()
    {   
        // Move camera into position
    }

    // Update is called once per frame
    void Update()
    {
        var x = Input.GetAxis("Horizontal");
        var y = Input.GetAxis("Vertical");
        var z = Input.GetAxis("Mouse ScrollWheel");
        var btn = Input.GetButton("Fire1");

        //handle zoom

        // Translate or Rotate?
        if (btn) 
        {
            // Translate
            
            var angleCompensatedDelta = Quaternion.AngleAxis(HorizontalAngle, Vector3.up) * new Vector3(-1 * y,0,x);
            var delta = OffsetDeltaBounding(angleCompensatedDelta);
            Offset += delta;
            this.transform.position += delta;
        }
        else
        {
            // Horizontal Rotation
            var hAngle = -1 * x; // deg
            HorizontalAngle += hAngle;
            transform.RotateAround(TargetObject.transform.position + Offset, Vector3.up, hAngle);

            // Rotate Vertical
            var vAngle = VerticalAngleBounding(y); //deg
            VerticalAngle += vAngle;
            //var zDelta = Mathf.Cos(Mathf.Deg2Rad * VerticalAngle) * Zoom;
            //var vDelta = Mathf.Tan(Mathf.Deg2Rad * VerticalAngle) * zDelta;
            transform.RotateAround(TargetObject.transform.position + Offset, Quaternion.AngleAxis(HorizontalAngle, Vector3.up) * Vector3.forward, vAngle);

            this.transform.LookAt(TargetObject.transform.position + Offset);
        }
    }

    private Vector3 OffsetDeltaBounding(Vector3 delta)
    {
        var temp = Offset + delta;

        var rad = Mathf.Sqrt(Mathf.Pow(temp.x,2) + Mathf.Pow(temp.z,2));
        var normTemp = temp / rad;

        if (rad > MaxOffset) 
        {
            delta.x -= normTemp.x * (rad - MaxOffset);
            delta.z -= normTemp.z * (rad - MaxOffset);
        }

        return delta;
    }

    private float VerticalAngleBounding(float angle)
    {
        var temp = VerticalAngle + angle;
        if (temp > MaxVerticalRotation)
        {
            angle -= (temp - MaxVerticalRotation);
        }
        else if (temp < MinVerticalRotation)
        {
            angle -= (temp - MinVerticalRotation);
        }
        return angle;
    }
}
