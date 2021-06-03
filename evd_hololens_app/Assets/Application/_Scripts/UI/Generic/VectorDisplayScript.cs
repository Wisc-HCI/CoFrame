using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class VectorDisplayScript : MonoBehaviour
{
    public Text nameField;
    public Text xField;
    public Text yField;
    public Text zField;

    public string nameValue = "";
    private Vector3 _data;

    private void Awake()
    {
        nameField.text = nameValue;
        xField.text = data.x.ToString();
        yField.text = data.y.ToString();
        zField.text = data.z.ToString();
    }

    public Vector3 data
    {
        get
        {
            return _data;
        }

        set
        {
            if (_data != value)
            {
                _data = value;
                xField.text = _data.x.ToString();
                yField.text = _data.y.ToString();
                zField.text = _data.z.ToString();
            }
        }
    }
}
