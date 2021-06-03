using UnityEngine;
using UnityEngine.UI;

public class PrimitiveScript : MonoBehaviour
{
    public string type;
    public string uuid;

    public GameObject interactivePanel;

    public System.Action<string> primitiveClickCallback = null;
    public System.Action<string> deleteClickCallback = null;
    public System.Action<string> shiftLeftClickCallback = null;
    public System.Action<string> shiftRightClickCallback = null;

    private Text textField;

    public void OnInstantiate(string type, string uuid, string name)
    {
        this.type = type;
        this.uuid = uuid;
        textField = GetComponentInChildren<Text>();
        textField.text = name;
    }

    public void OnPrimitiveClick()
    {
        primitiveClickCallback(uuid);
    }

    public void OnDeleteClick()
    {
        deleteClickCallback(uuid);
    }

    public void OnShiftLeftClick()
    {
        shiftLeftClickCallback(uuid);
    }

    public void OnShiftRightClick()
    {
        shiftRightClickCallback(uuid);
    }

    public void ActivePrimitive(bool status)
    {
        interactivePanel.SetActive(status);
    }
}
