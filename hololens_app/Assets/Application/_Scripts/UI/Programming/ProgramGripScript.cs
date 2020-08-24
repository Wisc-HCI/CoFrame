using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using Cobots;

public class ProgramGripScript : MonoBehaviour
{
    /*
    public ViewScript view;

    public float defaultPosition = 0.0f;
    public float defaultEffort = 0.0f;
    public float defaultSpeed = 0.0f;

    public Text positionText;
    public Slider positionSlider;
    public Text effortText;
    public Slider effortSlider;
    public Text speedText;
    public Slider speedSlider;

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
                var tmp = view.app.GetPrimitive(view.activeUuid);
                var prm = (GripPrimitive)(view.app.GetPrimitive(view.activeUuid));

                positionSlider.value = prm.position;
                positionText.text = string.Format("({0} %)", prm.position);

                effortSlider.value = prm.effort;
                effortText.text = string.Format("({0} %)", prm.effort);

                speedSlider.value = prm.speed;
                speedText.text = string.Format("({0} %)", prm.speed);
            }
            else
            {
                positionSlider.value = defaultPosition;
                positionText.text = string.Format("({0} %)", defaultPosition);

                effortSlider.value = defaultEffort;
                effortText.text = string.Format("({0} %)", defaultEffort);

                speedSlider.value = defaultSpeed;
                speedText.text = string.Format("({0} %)", defaultSpeed);
            }
        }
    }

    public void OpenButtonCallback()
    {
        positionSlider.value = 0.0f;
        PositionValueChangedCallback();
    }

    public void CloseButtonCallback()
    {
        positionSlider.value = 100.0f;
        PositionValueChangedCallback();
    }

    public void PositionValueChangedCallback()
    {
        positionText.text = string.Format("({0} %)", positionSlider.value);
        if (view.activeUuid != null)
        {
            view.app.SetPositionGripPrimitive(view.activeUuid, positionSlider.value);
        }
    }

    public void EffortValueChangedCallback()
    {
        effortText.text = string.Format("({0} %)", effortSlider.value);
        if (view.activeUuid != null)
        {
            view.app.SetEffortGripPrimitive(view.activeUuid, effortSlider.value);
        }
    }

    public void SpeedValueChangedCallback()
    {
        speedText.text = string.Format("({0} %)", speedSlider.value);
        if (view.activeUuid != null)
        {
            view.app.SetSpeedGripPrimitive(view.activeUuid, speedSlider.value);
        }
    }

    */
}
