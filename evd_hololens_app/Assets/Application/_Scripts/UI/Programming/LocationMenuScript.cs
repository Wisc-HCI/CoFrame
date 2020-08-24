using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class LocationMenuScript : MonoBehaviour
{
    public Button MenuButton;
    public GameObject MenuObject;

    public Button CloseButton;
    public Button DeleteButton;
    public GameObject ActionObject;

    public Button ConfirmButton;
    public Button CancelButton;
    public GameObject DeleteObject;

    public void MenuButtonPressed()
    {
        MenuObject.SetActive(false);
        ActionObject.SetActive(true);
    }

    public void CloseButtonPressed()
    {
        MenuObject.SetActive(true);
        ActionObject.SetActive(false);
    }

    public void DeleteButtonPressed()
    {
        DeleteObject.SetActive(true);
        ActionObject.SetActive(false);
    }

    public void ConfirmButtonPressed()
    {
        //TODO delete routine
    }

    public void CancelButtonPressed()
    {
        DeleteObject.SetActive(false);
        ActionObject.SetActive(true);
    }
}
