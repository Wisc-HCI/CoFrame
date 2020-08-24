using UnityEngine;
using UnityEngine.UI;

public class YesNoPopupMessageScript : MonoBehaviour
{
    public Text title;
    public Text message;
    public string defaultTitleText;

    private System.Action<string> callback;

    public void DisplayMessage(string message, System.Action<string> cb = null, string title = null)
    {
        this.title.text = title != null ? title : defaultTitleText;
        this.message.text = message;
        callback = cb;
        gameObject.SetActive(true);
    }

    public void OnYes()
    {
        gameObject.SetActive(false);
        callback?.Invoke("yes");
    }

    public void OnNo()
    {
        gameObject.SetActive(false);
        callback?.Invoke("no");
    }
}
