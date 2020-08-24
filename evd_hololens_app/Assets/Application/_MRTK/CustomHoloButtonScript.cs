using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CustomHoloButtonScript : MonoBehaviour
{
    [SerializeField]
    private GameObject background = null;

    public string identifier = "";

    public System.Action<string> callback = null;

    public void OnClick()
    {
        callback?.Invoke(identifier);
    }

    public bool backgroundVisible
    {
        get
        {
            return background.activeSelf;
        }

        set
        {
            if (background.activeSelf != value)
            {
                background.SetActive(value);
            }
        }
    } 
}
