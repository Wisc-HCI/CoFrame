using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class FuncPrimitvePanel : MonoBehaviour
{
    public float x;
    public float y;
    RectTransform rtPanelBtn;
    private float widthOfFuncWithSpacing;
    public List<Transform> ListOfPrimitves;
  
    // Start is called before the first frame update
    void Start()
    {
        rtPanelBtn = (RectTransform)this.gameObject.transform;
        x = rtPanelBtn.rect.x;
        y = rtPanelBtn.rect.y;
        widthOfFuncWithSpacing = 120;
        for (int i = 0; i < rtPanelBtn.childCount; i++)
        {
            ListOfPrimitves.Add(rtPanelBtn.GetChild(i));
        }
        for(int j = 0; j < ListOfPrimitves.Count; j++)
        {
            ListOfPrimitves[j].localPosition = new Vector3(x+50,y+40+(j*widthOfFuncWithSpacing),0);
        }

    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
