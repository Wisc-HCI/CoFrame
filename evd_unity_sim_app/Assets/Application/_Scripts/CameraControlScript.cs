using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraControlScript : MonoBehaviour
{
    public GameObject TargetObject;

    public float MaxOffset = 2;

    public float MaxVerticalRotation = 44;
    public float MinVerticalRotation = -60;

    public float MaxZoom = 2;
    public float MinZoom = 0.1f;

    public Vector2 TranslateSpeedScale = new Vector2(0.25f,0.25f);
    public Vector2 RotateSpeedScale = new Vector2(5,5);
    public float ZoomSpeedScale = 1.5f;

    private Vector3 Offset = new Vector3();

    private float VerticalAngle = 0;
    private float HorizontalAngle = 0;

    private float Zoom = 1;

    void Start() {
        #if !UNITY_EDITOR && UNITY_WEBGL
            WebGLInput.captureAllKeyboardInput = false;
        #endif    
    }

    // Update is called once per frame
    void Update()
    {
        var x = Input.GetAxis("Horizontal");
        var y = Input.GetAxis("Vertical");
        var z = Input.GetAxis("Mouse ScrollWheel");
        var btn = Input.GetButton("Fire1");

        //handle zoom
        var zoom = ZoomBounding(-1 * z);
        Zoom += zoom;
        var move = (TargetObject.transform.position + Offset) - this.transform.position;
        var dist = Mathf.Sqrt(Mathf.Pow(move.x,2) + Mathf.Pow(move.y,2) + Mathf.Pow(move.z,2));
        var dif = dist - Zoom;
        var normMove = move / dist;
        this.transform.position = (normMove * dif) + this.transform.position;

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
            var hAngle = HorizontalAngleBounding(x); // deg
            HorizontalAngle += hAngle;
            transform.RotateAround(TargetObject.transform.position + Offset, Vector3.up, hAngle);

            // Rotate Vertical
            var vAngle = VerticalAngleBounding(-1 * y); //deg
            VerticalAngle += vAngle;
            transform.RotateAround(TargetObject.transform.position + Offset, Quaternion.AngleAxis(HorizontalAngle, Vector3.up) * Vector3.forward, vAngle);

            this.transform.LookAt(TargetObject.transform.position + Offset);
        }
    }

    private Vector3 OffsetDeltaBounding(Vector3 delta)
    {
        delta.x *= TranslateSpeedScale.x;
        delta.z *= TranslateSpeedScale.y;

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
        angle *= RotateSpeedScale.y;

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

    private float HorizontalAngleBounding(float angle) 
    {
        angle *= RotateSpeedScale.x;
        return angle;
    }

    private float ZoomBounding(float zoom)
    {
        zoom *= ZoomSpeedScale;

        var temp = Zoom + zoom;
        if (temp > MaxZoom)
        {
            zoom -= (temp - MaxZoom);
        }
        else if (temp < MinZoom)
        {
            zoom -= (temp - MinZoom);
        }

        return zoom;
    }
}
