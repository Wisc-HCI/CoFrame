using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class YogeshWaypointCode : MonoBehaviour
{
    /*
    public Vector3 KeyLocation1;
    public Vector3 KeyLocation2;
    int dx = 10;
    GameObject Waypoint;

    public Vector3 GetPositionOfWaypoint(float LineParameter_t)
    {
        Vector3 Waypoint;
        Vector3 DirectionVector;
        DirectionVector = KeyLocation2 - KeyLocation1;
        Waypoint.x = KeyLocation1.x + DirectionVector.x * LineParameter_t;
        Waypoint.y = KeyLocation1.y + DirectionVector.y * LineParameter_t;
        Waypoint.z = KeyLocation1.z + DirectionVector.z * LineParameter_t;
        return Waypoint;
    }

    // Start is called before the first frame update
    public void GetWaypoints()
    {
        Vector3 positionOfWaypoint;
        int i;
        int noOfWaypoints = (100 / dx) - 1; ;

        for( i = 1; i <= noOfWaypoints; i++)
        {
            positionOfWaypoint = GetPositionOfWaypoint(((float)dx/100)*i);
            Debug.Log("Waypoint : " + i + " x : " + positionOfWaypoint.x + " y : " + positionOfWaypoint.y + " z : " + positionOfWaypoint.z);
        }
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }
    */

    /*
    public Material EditColour;
    public GameObject TrajectorySphere;
    public Button EditButton;
    //Edit the parameters related to a waypoint. 
    public void Edit()
    {
        TrajectorySphere.GetComponent<Renderer>().material = EditColour;

    }
    // Start is called before the first frame update
    void Start()
    {
        TrajectorySphere = this.transform.parent.Find("Trajectory1").gameObject;
        EditButton = this.transform.Find("Edit").GetComponent<Button>();
        Button btn = EditButton;
        btn.onClick.AddListener(Edit);
    }

    // Update is called once per frame
    void Update()
    {

    }
    */
}
