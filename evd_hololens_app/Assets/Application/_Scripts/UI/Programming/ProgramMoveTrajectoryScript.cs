using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using Cobots;

public class ProgramMoveTrajectoryScript : MonoBehaviour
{
    /*
    public ViewScript view;

    public InputField nameField;
    public Slider velocityField;
    public Slider accelerationField;
    public Dropdown typeField;

    public Text velocityValue;
    public Text accelerationValue;

    public float defaultVelocity = 1.0f;
    public float defaultAcceleration = 1.0f;
    public string defaultType = "joint";

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
                var traj = view.app.GetTrajectory(view.activeUuid);
                if (traj.name != null)
                {
                    nameField.text = traj.name;
                }
                else
                {
                    nameField.text = "";
                    NameChangedCallback();
                }

                velocityField.value = traj.velocity;
                velocityValue.text = "(" + traj.velocity.ToString() + " %)";
                accelerationField.value = traj.acceleration;
                accelerationValue.text = "(" + traj.acceleration.ToString() + " %)";
                typeField.value = IndexFromType(traj.type);
            }
            else
            {
                nameField.text = "";
                velocityField.value = defaultVelocity;
                velocityValue.text = "(" + defaultVelocity.ToString() + " %)";
                accelerationField.value = defaultAcceleration;
                accelerationValue.text = "(" + defaultAcceleration.ToString() + " %)";
                typeField.value = IndexFromType(defaultType);
            }
        }
    }

    private int IndexFromType(string type)
    {
        int retVal = -1;
        for (int i=0; i<typeField.options.Count; i++)
        {
            if (typeField.options[i].text == type)
            {
                retVal = i;
                break;
            }
        }
        return retVal;
    }

    public void NameChangedCallback()
    {
        if (view.activeUuid != null)
            view.app.SetTrajectoryName(view.activeUuid, nameField.text);
    }

    public void VelocityChangedCallback()
    {
        velocityValue.text = "(" + velocityField.value.ToString() + " %)";
        if (view.activeUuid != null)
            view.app.SetTrajectoryVelocity(view.activeUuid, velocityField.value);
    }

    public void AccelerationChangedCallback()
    {
        accelerationValue.text = "(" + accelerationField.value.ToString() + " %)";
        if (view.activeUuid != null)
            view.app.SetTrajectoryAcceleration(view.activeUuid, accelerationField.value);
    }

    public void TypeChangedCallback()
    {
        if (view.activeUuid != null)
            view.app.SetTrajectoryType(view.activeUuid, typeField.options[typeField.value].text);
    }
    */
}
