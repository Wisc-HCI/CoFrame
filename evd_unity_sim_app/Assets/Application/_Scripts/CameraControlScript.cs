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

    private float ZoomVal = 1;

    private string activeControl = "rotate";

    void Start() {
        #if !UNITY_EDITOR && UNITY_WEBGL
            WebGLInput.captureAllKeyboardInput = false;
        #endif    
    }

    // Update is called once per frame
    void Update()
    {
        var x = Input.GetAxis("Mouse X");
        var y = Input.GetAxis("Mouse Y");
        var z = Input.GetAxis("Mouse ScrollWheel");
        var btn = Input.GetButton("Fire1");

        if (btn) 
        {
            switch(activeControl) 
            {
                case "rotate":
                    RotateControl(x,y);
                    ZoomControl(z);
                    break;
                case "translate":
                    TranslateControl(x,y);
                    ZoomControl(z);
                    break;
                default:
                    activeControl = "rotate";
                    break;
            }
        }
    }

    public void SetView(float z, float tx, float ty, float rx, float ry) 
    {
        // Set Raw Zoom
        if (z > MaxZoom)
        {
            z -= (z - MaxZoom);
        }
        else if (z < MinZoom)
        {
            z -= (z - MinZoom);
        }

        ZoomVal = z;
        var move = (TargetObject.transform.position + Offset) - this.transform.position;
        var dist = Mathf.Sqrt(Mathf.Pow(move.x,2) + Mathf.Pow(move.y,2) + Mathf.Pow(move.z,2));
        var dif = dist - ZoomVal;
        var normMove = move / dist;
        this.transform.position = (normMove * dif) + this.transform.position;

        // Set Raw Translate
        Vector3 rawTranslate = new Vector3(tx,0,ty);

        var rad = Mathf.Sqrt(Mathf.Pow(rawTranslate.x,2) + Mathf.Pow(rawTranslate.z,2));
        var normTemp = rawTranslate / rad;

        if (rad > MaxOffset) 
        {
            normTemp.x -= normTemp.x * (rad - MaxOffset);
            normTemp.z -= normTemp.z * (rad - MaxOffset);
        }

        Offset = normTemp;
        this.transform.position = normTemp;

        // Set Raw Rotate
        var deltaRx = HorizontalAngle - rx;
        HorizontalAngle = rx;
        transform.RotateAround(TargetObject.transform.position + Offset, Vector3.up, deltaRx);

        if (ry > MaxVerticalRotation)
        {
            ry -= (ry - MaxVerticalRotation);
        }
        else if (ry < MinVerticalRotation)
        {
            ry -= (ry - MinVerticalRotation);
        }

        var deltaRy = VerticalAngle - ry;
        VerticalAngle = ry;
        transform.RotateAround(TargetObject.transform.position + Offset, Quaternion.AngleAxis(HorizontalAngle, Vector3.up) * Vector3.forward, deltaRy);

        this.transform.LookAt(TargetObject.transform.position + Offset);
    }

    public void ChangeActiveControl(string controlName) 
    {
        activeControl = controlName;
    }

    private void ZoomControl(float z) 
    {
        //handle zoom
        var zoom = ZoomBounding(-1 * z);
        ZoomVal += zoom;
        var move = (TargetObject.transform.position + Offset) - this.transform.position;
        var dist = Mathf.Sqrt(Mathf.Pow(move.x,2) + Mathf.Pow(move.y,2) + Mathf.Pow(move.z,2));
        var dif = dist - ZoomVal;
        var normMove = move / dist;
        this.transform.position = (normMove * dif) + this.transform.position;
    }

    private void TranslateControl(float x, float y) 
    {
        // Translate
        var angleCompensatedDelta = Quaternion.AngleAxis(HorizontalAngle, Vector3.up) * new Vector3(-1 * y,0,x);
        var delta = OffsetDeltaBounding(angleCompensatedDelta);
        Offset += delta;
        this.transform.position += delta;
    }

    private void RotateControl(float x, float y) 
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

        var temp = ZoomVal + zoom;
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
