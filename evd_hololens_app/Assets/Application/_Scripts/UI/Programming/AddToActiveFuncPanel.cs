using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AddToActiveFuncPanel : MonoBehaviour
{
    public GameObject Move_Prefab;
    public GameObject ActiveFunctionScroll;
    private int ListOfFuncsActive;
    public void CreateNewFuncObject()
    {
        GameObject FuncButton = Instantiate(Move_Prefab);
        FuncButton.transform.SetParent(ActiveFunctionScroll.transform);
        int minimumXSpacing = 20;
        RectTransform rectBtn = (RectTransform)FuncButton.transform;
        RectTransform rectPanel = (RectTransform)ActiveFunctionScroll.transform;
        float widthOfFuncWithSpacing = rectBtn.rect.width;
        ListOfFuncsActive = ActiveFunctionScroll.GetComponent<ActiveFunctions>().ListOfFunctions.Count;
        FuncButton.transform.localScale = new Vector3(1,1,1);
        FuncButton.transform.localPosition = new Vector3(-480 + minimumXSpacing + (ListOfFuncsActive * 120) + minimumXSpacing, rectPanel.rect.y + 50, 0);
        Debug.Log(rectPanel.rect.x + minimumXSpacing + (ListOfFuncsActive * widthOfFuncWithSpacing) + minimumXSpacing);
        Debug.Log(rectPanel.rect.x);
        Debug.Log((ListOfFuncsActive));
        Debug.Log(widthOfFuncWithSpacing);
        var button = GetComponent<UnityEngine.UI.Button>();
        button.onClick.AddListener(() => FooOnClick());
    }

    void FooOnClick()
    {
        Debug.Log("Ta Da!");
    }
    // Start is called before the first frame update
    //void Start()
    //{
        
    //}

    //// Update is called once per frame
    //void Update()
    //{
        
    //}
}
