using UnityEngine;
using UnityEngine.UI;

public class TrajectoryScript : MonoBehaviour
{
    public string uuid;

    public GameObject interactivePanel;
    public GameObject useGrayout;

    public System.Action<string> selectCallback = null;
    public System.Action<string> deleteClickCallback = null;
    public System.Action<string> shiftLeftClickCallback = null;
    public System.Action<string> shiftRightClickCallback = null;
    public System.Action<string> useClickCallback = null;

    private Text textField;

    public void OnInstantiate(string text, string uuid)
    {
        this.uuid = uuid;
        textField = GetComponentInChildren<Text>();
        textField.text = text;
    }

    public void OnSelectClick()
    {
        selectCallback(uuid);
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

    public void ActiveTrajectory(bool status)
    {
        interactivePanel.SetActive(status);
    }

    public void UseTrajectory(bool state)
    {
        useGrayout.SetActive(state);
    }

    public void OnUseClick()
    {
        useClickCallback(uuid);
    }
}
  