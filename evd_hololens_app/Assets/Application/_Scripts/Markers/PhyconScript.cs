using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class PhyconScript : MonoBehaviour
{
    public GameObject panel;
    public Text message;

    private bool state = false;

    public void IconClickCallback()
    {
        state = !state;
        panel.SetActive(state);
    }

    public void DismissCallback()
    {
        Destroy(gameObject);
    }

}
