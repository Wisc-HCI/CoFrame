using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class Basic_Funtion : MonoBehaviour
{
    public float X;
    public float Y;
    public float Z;
    public float widthOfFuncBtn;
    public float heightOfFuncBtn;
    public Button FuncButton;
    public Quaternion rotation;
    public void SetText(string text)
    {
        Text txt = transform.Find("Text").GetComponent<Text>();
        txt.text = text;
    }
    public void MoveArrows()
    {
        Transform LeftArrow = this.gameObject.transform.GetChild(1);
        Transform RightArrow = this.gameObject.transform.GetChild(2);
        LeftArrow.localPosition = new Vector3(0-(widthOfFuncBtn/2)-5, 0,0);
        RightArrow.localPosition = new Vector3(0+(widthOfFuncBtn/2)+5, 0, 0);
        LeftArrow.localScale = new Vector3(15,15,15);
        RightArrow.localScale = new Vector3(15,15,15);
        rotation = Quaternion.Euler(0,0,90);
        LeftArrow.localRotation = rotation;
        rotation = Quaternion.Euler(0, 0, 270);
        RightArrow.localRotation = rotation;
    }
    void Start()
    {
        RectTransform rtBtn = (RectTransform)this.gameObject.transform;
        //Button btn = (Button)this.gameObject;
        //btn.onClick.AddListener(MoveArrows);
        X = this.gameObject.transform.localPosition.x;
        Y = this.gameObject.transform.localPosition.x;
        widthOfFuncBtn = rtBtn.rect.width;
        heightOfFuncBtn = rtBtn.rect.height;
        Debug.Log("x: " + X);
        Debug.Log("Width: " + widthOfFuncBtn);


        print("Unity is Running");
    }
  
}
