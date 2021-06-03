using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class StandardToolsLayoutScript : MonoBehaviour
{
    public GameObject backgroundExpanded;
    public GameObject backgroundCollapsed;
    public GameObject[] collapsableButtons;

    [SerializeField]
    private bool _displayButtons = false;

    private void Awake()
    {
        UpdateVisuals();
    }

    public bool displayButtons
    {
        get
        {
            return _displayButtons;
        }

        set
        {
            if (_displayButtons != value)
            {
                _displayButtons = value;
                UpdateVisuals();
            }
        }
    } 

    private void UpdateVisuals()
    {
        foreach (var go in collapsableButtons)
        {
            go.SetActive(displayButtons);
        }

        backgroundExpanded.SetActive(displayButtons);
        backgroundCollapsed.SetActive(!displayButtons);
    }
}
