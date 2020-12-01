using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using EvD;

public class ProgramMachineScript : MonoBehaviour
{
    /*
    public ViewScript view;
    public Dropdown actionField;
    public Text actionDescription;

    private void Awake()
    {
        actionField.AddOptions(new List<string>(MachinePrimitive.ACTIONS));

        view = GetComponent<ViewScript>();
        view.visibleCallbacks.Add(Visible);
    }

    public void Visible(bool val)
    {
        if (val)
        {
            if (view.activeUuid != null)
            {
                var prm = (MachinePrimitive)(view.app.GetPrimitive(view.activeUuid));
                int index = -1;
                for (int i=0; i<actionField.options.Count; i++)
                {
                    if (actionField.options[i].text == prm.action)
                    {
                        index = i;
                    }
                }

                if (index < 0)
                {
                    actionField.value = 0;
                    ActionValueChanged();
                }
                else
                {
                    actionField.value = index;
                    actionDescription.text = MachinePrimitive.ACTION_DESCRIPTIONS[actionField.value];
                }

            }
        }
    }

    public void ActionValueChanged()
    {
        actionDescription.text = MachinePrimitive.ACTION_DESCRIPTIONS[actionField.value];

        if (view.activeUuid != null)
        {
            view.app.SetActionMachinePrimitive(view.activeUuid, actionField.options[actionField.value].text);
        }
    }
    */
}
