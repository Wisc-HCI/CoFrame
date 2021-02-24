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
    }
}
