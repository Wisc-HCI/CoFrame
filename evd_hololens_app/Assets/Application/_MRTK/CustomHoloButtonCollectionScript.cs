using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CustomHoloButtonCollectionScript : MonoBehaviour
{
    public CustomHoloButtonScript[] buttons { get; private set; }
    public string[] identifiers { get; private set; }

    private void Awake()
    {
        buttons = GetComponentsInChildren<CustomHoloButtonScript>();
        identifiers = new string[buttons.Length];
        for (int i = 0; i < buttons.Length; i++)
        {
            identifiers[i] = buttons[i].identifier;
        }
    }

    public bool visible
    {
        get
        {
            return gameObject.activeSelf;
        }

        set
        {
            if (gameObject.activeSelf != value)
            {
                gameObject.SetActive(value);
            }
        }
    }

    public void SetBackgroundVisible(string identifier, bool value)
    {
        bool inList = false;
        foreach (var b in buttons)
        {
            if (b.identifier == identifier)
            {
                b.backgroundVisible = value;
                inList = true;
                break;
            }
        }

        if (!inList)
        {
            throw new System.Exception("identifier not in button list");
        }
    }

    public bool GetBackgroundVisible(string identifier)
    {
        foreach (var b in buttons)
        {
            if (b.identifier == identifier)
            {
                return b.backgroundVisible;
            }
        }

        throw new System.Exception("identifier not in button list");
    }
}
