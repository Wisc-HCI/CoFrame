using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class FloatSliderScript : MonoBehaviour
{
    public Slider valueSlider;
    public Text nameLabel;
    public Text leftLabel;
    public Text rightLabel;
    public Text valueEditText;
    public InputField valueField;

    public System.Action<string, float> valueChangedCallback = null; // name, value

    [SerializeField]
    private string _name = "";
    [SerializeField]
    private string _units = "%";
    [SerializeField]
    private float _minValue = 0.0f;
    [SerializeField]
    private float _maxValue = 100.0f;
    [SerializeField]
    private float _defaultValue = 50.0f;

    private bool modified = false;

    private void Awake()
    {
        nameLabel.text = name;

        valueSlider.minValue = minValue;
        valueSlider.maxValue = maxValue;
        valueSlider.value = defaultValue;

        leftLabel.text = string.Format("{0} {1}", minValue, units);
        rightLabel.text = string.Format("{0} {1}", maxValue, units);
        valueEditText.text = string.Format("{0} {1}", valueSlider.value, units);
        valueField.text = valueSlider.value.ToString();

        valueEditText.gameObject.SetActive(true);
        valueField.gameObject.SetActive(false);
    }

    public new string name
    {
        get
        {
            return _name;
        }

        set
        {
            if (_name != value)
            {
                _name = value;
                nameLabel.text = _name;
            }
        }
    }

    public float value
    {
        get
        {
            return valueSlider.value;
        }

        set
        {
            if (valueSlider.value != value)
            {
                valueSlider.value = value;
                modified = true;
                valueEditText.text = string.Format("{0} {1}", valueSlider.value, units);
                valueField.text = valueSlider.value.ToString();
            }
        }
    }

    public float minValue
    {
        get
        {
            return _minValue;
        }

        set
        {
            if (_minValue != value)
            {
                _minValue = value;
                valueSlider.minValue = _minValue;
                leftLabel.text = string.Format("{0} {1}", _minValue, units);
            }
        }
    }

    public float maxValue
    {
        get
        {
            return _maxValue;
        }

        set
        {
            if (_maxValue != value)
            {
                _maxValue = value;
                valueSlider.maxValue = _maxValue;
                rightLabel.text = string.Format("{0} {1}", _maxValue, units);
            }
        }
    }

    public float defaultValue
    {
        get
        {
            return _defaultValue;
        }

        set
        {
            if (_defaultValue != value)
            {
                _defaultValue = value;

                if (!modified)
                {
                    valueSlider.value = _defaultValue;
                    valueEditText.text = string.Format("{0} {1}", valueSlider.value, units);
                    valueField.text = valueSlider.value.ToString();
                }
            }
        }
    }

    public string units
    {
        get
        {
            return _units;
        }

        set
        {
            if (_units != value)
            {
                _units = value;
                leftLabel.text = string.Format("{0} {1}", minValue, _units);
                rightLabel.text = string.Format("{0} {1}", maxValue, _units);
                valueEditText.text = string.Format("{0} {1}", valueSlider.value, units);
            }
        }
    }

    public void OnValueChange()
    {
        modified = true;
        valueEditText.text = string.Format("{0} {1}", valueSlider.value, units);
        valueField.text = valueSlider.value.ToString();

        valueChangedCallback?.Invoke(name, valueSlider.value);
    }

    public void OnButtonPress()
    {
        valueEditText.gameObject.SetActive(false);
        valueField.gameObject.SetActive(true);
    }

    public void OnFieldEdit()
    {
        valueEditText.gameObject.SetActive(true);
        valueField.gameObject.SetActive(false);

        // format string to float
        string str = valueField.text;
        if (str == "")
        {
            str = "0";
        }
        float val = float.Parse(str);
        if (val > maxValue)
        {
            val = maxValue;
        }
        else if (val < minValue)
        {
            val = minValue;
        }

        // store values
        valueSlider.value = val;
        modified = true;
        valueEditText.text = string.Format("{0} {1}", val, units);
        valueField.text = val.ToString();

        valueChangedCallback?.Invoke(name, val);
    }
}
