using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class TrajectoryManager : MonoBehaviour
{
    public List<Transform> ListOfTrajectories = new List<Transform>();
    public Button TrajectoryBtn;
    public Button TrajBtn;
    public float widthOfPanel;
    public float heightOfPanel;
    //private float ycenterOfTrajsPanel;
    private float yPrevTraj;
    // Start is called before the first frame update

    void Start()
    {
        //Has to be set as TrajectoryBtn = prefab of trajbtn
        RectTransform rt = (RectTransform)this.gameObject.transform;
        widthOfPanel = rt.rect.width;
        heightOfPanel = rt.rect.height;
        for (int i = 0; i < this.gameObject.transform.childCount-1; i++)
        {
            ListOfTrajectories.Add(this.gameObject.transform.GetChild(i));
            TrajBtn = ListOfTrajectories[i].GetComponent<Button>();
            TrajBtn.onClick.AddListener(TrajectorySelected);
        }
        

    }
    public void TrajectorySelected()
    {

    }
   public void AddTrajectory()
    {
        //ycenterOfTrajsPanel = 300;
        Button newTrajBtn = Instantiate(TrajectoryBtn);
        //newTrajBtn.GetComponent<Trajectory>();
        //Get the y position of the last child
        yPrevTraj = ListOfTrajectories[ListOfTrajectories.Count-1].localPosition.y;
        newTrajBtn.transform.SetParent(this.transform);
        newTrajBtn.transform.localPosition = new Vector3(0, yPrevTraj-30-5, 0);
        //newTrajBtn.GetComponentInChildren() = "TRAJECTORY " + ListOfTrajectories.Count;
        ListOfTrajectories.Add(newTrajBtn.transform);
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
