using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class WaypointMenuScript : MonoBehaviour
{
    public Button MenuButton;
    public GameObject MenuObject;

    public Button CloseButton;
    public Button ParameterButton;
    public Button DeleteButton;
    public GameObject ActionObject;

    public Button ConfirmButton;
    public Button CancelButton;
    public GameObject DeleteObject;

    public Button SaveButton;
    public Button BackButton;
    public GameObject ParameterObject;

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

    public void ParameterButtonPressed()
    {
        ActionObject.SetActive(false);
        ParameterObject.SetActive(true);
    }

    public void SaveButtonPressed()
    {
        // copy values into object structure

        ActionObject.SetActive(true);
        ParameterObject.SetActive(false);
    }

    public void BackButtonPressed()
    {
        ActionObject.SetActive(true);
        ParameterObject.SetActive(false);
    }
}
