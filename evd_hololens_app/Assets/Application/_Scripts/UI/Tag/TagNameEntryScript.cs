using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class TagNameEntryScript : MonoBehaviour
{
    private string _name = "";
    private bool _isInteractable = false;
    private bool _isVisible = false;

    public Text text;
    public Button button;
    public InputField inputField;
    public GameObject background;

    public string EMPTY_DISPLAY_NAME = "<Name>";
    public float TEXT_WIDTH_SCALAR = 10.0f;
    public float TEXT_ENTRY_MIN_WIDTH = 200.0f;
    public float X_PADDING = 10.0f;

    public System.Action<string> nameUpdatedCallback = null;

    private bool state = false;

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

                text.text = value;
                inputField.text = value;

                updateObjectTextWidth(button.GetComponent<RectTransform>(), text.text, 0, !state);
                updateObjectTextWidth(inputField.GetComponent<RectTransform>(), inputField.text, TEXT_ENTRY_MIN_WIDTH, state);
            }
        }
    }

    public bool isInteractable
    {
        get
        {
            return _isInteractable;
        }

        set
        {
            if (_isInteractable != value)
            {
                _isInteractable = value;


                if (_isInteractable)
                {
                    button.enabled = true;
                }
                else
                {
                    button.gameObject.SetActive(true);
                    inputField.gameObject.SetActive(false);

                    button.enabled = false;
                }
            }
        }
    }

    public bool isVisible
    {
        get
        {
            return _isVisible;
        }

        set
        {
            if (_isVisible != value)
            {
                _isVisible = value;

                if (_isVisible)
                {
                    button.gameObject.SetActive(!state);
                    inputField.gameObject.SetActive(state);
                }
                else
                {
                    button.gameObject.SetActive(false);
                    inputField.gameObject.SetActive(false);
                }
            }
        }
    }

    private void Start()
    {
        // set initial visibility
        if (isVisible)
        {
            button.gameObject.SetActive(true);
            inputField.gameObject.SetActive(false);
        }
        else
        {
            button.gameObject.SetActive(false);
            inputField.gameObject.SetActive(false);
        }

        // set initial interactability
        if (isInteractable)
        {
            button.enabled = true;
            state = false;
        }
        else
        {
            button.gameObject.SetActive(true);
            inputField.gameObject.SetActive(false);

            button.enabled = false;
        }

        // set initial name text
        text.text = getDisplayName();
        updateObjectTextWidth(button.GetComponent<RectTransform>(), text.text);
    }

    public void ButtonPressed()
    {
        button.gameObject.SetActive(false);
        inputField.gameObject.SetActive(true);
        inputField.text = name;
        state = true;
        updateObjectTextWidth(inputField.GetComponent<RectTransform>(), inputField.text, TEXT_ENTRY_MIN_WIDTH);
    }

    public void ValueChange()
    {
        name = inputField.text;
    }

    public void EndEdit()
    {
        button.gameObject.SetActive(true);
        inputField.gameObject.SetActive(false);
        text.text = getDisplayName();
        state = false;
        updateObjectTextWidth(button.GetComponent<RectTransform>(), text.text);

        nameUpdatedCallback?.Invoke(name);
    }

    private string getDisplayName()
    {
        if (name == "")
        {
            return EMPTY_DISPLAY_NAME;
        }
        else
        {
            return name;
        }
    }

    private void updateObjectTextWidth(RectTransform rt, string str, float min = 0, bool updateBackground = true)
    {
        TextGenerator textGen = new TextGenerator();
        TextGenerationSettings generationSettings = text.GetGenerationSettings(rt.rect.size);
        float newWidth = textGen.GetPreferredWidth(str, generationSettings);

        if (newWidth < min)
        {
            newWidth = min;
        }
        rt.SetSizeWithCurrentAnchors(RectTransform.Axis.Horizontal, newWidth);

        if (updateBackground)
        {
            var s = background.transform.localScale;
            s.x = newWidth + X_PADDING;
            background.transform.localScale = s;
        }
    }
}
