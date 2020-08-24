using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ActiveFunctions : MonoBehaviour
{
    public List<Transform> ListOfFunctions = new List<Transform>();
    public float widthOfPanel;
    public float heightOfPanel;
    public float widthOfFuncButton;
    public float heightOfFuncButton;
    private float minimumXSpacing;
    public float x;
    public float y;
    public float z;
    private float widthOfFuncWithSpacing;
    // Start is called before the first frame update
    void Start()
    {
        
        RectTransform rt = (RectTransform) this.gameObject.transform;
        widthOfPanel = rt.rect.width;
        heightOfPanel = rt.rect.height;
        x = rt.rect.x;
        y = rt.rect.y;
        z = this.gameObject.transform.position.z;

        for (int i = 0; i < this.gameObject.transform.childCount;i++)
        {
            ListOfFunctions.Add(this.gameObject.transform.GetChild(i));
        }

        RectTransform FuncButton;
         minimumXSpacing = 20;
        widthOfFuncWithSpacing = 20 + 80 + 20;
        for(int i = 0; i < ListOfFunctions.Count; i++)
        {
            FuncButton = (RectTransform)ListOfFunctions[i];
            //Set Width of buttons
            //Set Height of buttons
            FuncButton.sizeDelta = new Vector2(80,80);
            //Set position of buttons
            FuncButton.localPosition = new Vector3(x+minimumXSpacing+(i*widthOfFuncWithSpacing)+minimumXSpacing,y+50,0);
        }

        
    }

    // Update is called once per frame
    void Update()
    {

        
    }
}
