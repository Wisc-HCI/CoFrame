using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using EvD;

public class ProgramDelayScript : MonoBehaviour
{
    /*
    public ViewScript view;

    public GameObject TimeOption;
    public GameObject UntilOption;

    public Dropdown typeField;
    public InputField delayField;

    public float defaultValue = 0.0f;
    public float step = 0.5f;

    private void Awake()
    {
        view = GetComponent<ViewScript>();
        view.visibleCallbacks.Add(Visible);
    }

    public void Visible(bool val)
    {
        if (val)
        {
            if (view.activeUuid != null)
            {
                var prm = (DelayPrimitive)(view.app.GetPrimitive(view.activeUuid));
                delayField.text = prm.duration.ToString();
            }
            else
            {
                delayField.text = defaultValue.ToString();
            }
        }
    }

    public void MinusButtonCallback()
    {
        float value = 0.0f;
        var result = float.TryParse(delayField.text, out value);
        delayField.text = (value - step).ToString();
        ValueChangedCallback();
    }

    public void PlusButtonCallback()
    {
        float value = 0.0f;
        var result = float.TryParse(delayField.text, out value);
        delayField.text = (value + step).ToString();
        ValueChangedCallback();
    }

    public void ValueChangedCallback()
    {
        if (view.activeUuid != null)
        {
            float value = 0.0f;
            var result = float.TryParse(delayField.text, out value);
            if (value < 0)
            {
                value = 0;
                delayField.text = value.ToString();
            }

            view.app.SetDurationDelayPrimitive(view.activeUuid, value);

            
        }
    }

    public void TypeDropdownCallback()
    {
        if (typeField.options[typeField.value].text == "Time")
        {

        }
        else if (typeField.options[typeField.value].text == "Until")
        {

        }
    }
    */
}
