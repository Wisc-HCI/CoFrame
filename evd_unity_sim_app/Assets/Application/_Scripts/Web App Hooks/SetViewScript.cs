using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SetViewScript : MonoBehaviour
{
    public new CameraControlScript camera;
    
    public void SetView(string data)
    {
        //TODO convert data (JSON string) into View commands
        Debug.Log(data);

        camera.SetView(1,0,0,0,0);
    }

    public void ChangeActiveControl(string controlName) 
    {
        camera.ChangeActiveControl(controlName);
    }
}
