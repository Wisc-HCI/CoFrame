using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class BreadcrumbItemScript : MonoBehaviour
{
    public Text titleField;
    public string id;
    public System.Action<string> callback = null;

    public void Initialize(string title, string id)
    {
        titleField.text = title;
        this.id = id;
    }

    public void OnButtonPressed()
    {
        callback?.Invoke(id);
    }
}
