using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class YogeshTrajectoryCode : MonoBehaviour
{
    /*
    public GameObject KeyLocation;
    LineRenderer line;
    CapsuleCollider unitCollider;
    // Start is called before the first frame update
    void Start()
    {
        float xPos;
        float yPos;
        float zPos;
        GameObject sphere;
        sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        //Create two key location spheres
        //Instantiate Key Location 1
        GameObject KeyLocation1 = Instantiate(KeyLocation,new Vector3(0,0,0),Quaternion.identity);
        //Instantiate Key Location 2
        GameObject KeyLocation2 = Instantiate(KeyLocation,new Vector3(0,10,0),Quaternion.identity);
        //Render a line between them
        line = GetComponent<LineRenderer>();
        line.SetPosition(0,KeyLocation1.transform.position);
        line.SetPosition(1,KeyLocation2.transform.position);
        int t = 4;
        //Now at every unit distance between the key locations, draw a shpere or an object
        xPos = KeyLocation1.transform.position.x + KeyLocation2.transform.position.x * t;
        yPos = KeyLocation1.transform.position.y + KeyLocation2.transform.position.y * t;
        zPos = KeyLocation1.transform.position.z + KeyLocation2.transform.position.z * t;
        //Draw a sphere or collider object at these points
        sphere.transform.position = new Vector3(xPos,yPos,zPos);
        //Attach a unit collider to these objects
        unitCollider = sphere.AddComponent<CapsuleCollider>();
        //To test, On mouse entry each collider must change color.
        //Attach the script to the spheres
        sphere.AddComponent<Trajectory_MultipleColliders>();
    }

    void OnMouseEnter()
    {
        Debug.Log("Here");
        Debug.Log(Input.mousePosition);
        //Vector3 pz = Camera.main.ScreenToWorldPoint(Input.mousePosition);
        //pz.z = 0;
        //gameObject.transform.position = pz;
        //// Change the color of the GameObject to red when the mouse is over GameObject
        //line.material = MouseOverColor;
    }

    void OnMouseExit()
    {
        // Reset the color of the GameObject back to normal
        //line.material = OriginalColor;
    }
    // Update is called once per frame
    void Update()
    {
        
    }
    */

    /*
    public GameObject TrajectorySphere;
    public Material Highlight;
    public float positionX;
    public float positionY;
    public float positionZ;
    public float velocity;
    public float effort;
    public float orientation;
    public GameObject Trajectory_Panel;
    // Start is called before the first frame update
    public void assignTrajectory(int trajectoryNo)
    {
        TrajectorySphere = this.transform.parent.parent.Find("Trajectory" + trajectoryNo.ToString()).gameObject;
    }
    public void assignValue()
    {
        positionX = 1;
        positionY = 2;
        positionZ = 3;
        velocity = 4;
        effort = 5;
        orientation = 6;
        GameObject InfoPanel = Instantiate(Trajectory_Panel);
        InfoPanel.transform.SetParent(TrajectorySphere.transform);
        InfoPanel.transform.localPosition = new Vector3(4, 7, 0);
        InfoPanel.transform.Find("X").GetComponent<Text>().text = positionX.ToString();
        InfoPanel.transform.Find("Y").GetComponent<Text>().text = positionY.ToString();
        InfoPanel.transform.Find("Z").GetComponent<Text>().text = positionZ.ToString();
        InfoPanel.transform.Find("VelValue").GetComponent<Text>().text = velocity.ToString();
        InfoPanel.transform.Find("EffValue").GetComponent<Text>().text = effort.ToString();
        InfoPanel.transform.Find("OrienValue").GetComponent<Text>().text = orientation.ToString();
        TrajectorySphere.GetComponent<Renderer>().material = Highlight;
        //InfoPanel.transform.FindChild("X").text = positio
    }
    */

    /*
    public Transform start;
    public Transform end;
    public Transform target;
    public Material OriginalColor;
    public Material MouseOverColor;

    public LinkedList<GameObject> Waypoints = new LinkedList<GameObject>();

    public const float Unit = 5;

    public Material WaypointColor;
    public GameObject Waypoint;

    LineRenderer line;
    CapsuleCollider capsule;

    public float LineWidth; // use the same as you set in the line renderer.

    void Start()
    {
        //Step 1: Get Key Locations
        //Get starting and final point.
        //Start - Starting location
        //End - End location
        Debug.Log("Hello__!");
        start.transform.localPosition = new Vector3(10, 10, 10);
        Debug.Log(start.transform.position);
        end.transform.localPosition = new Vector3(0, 0, 0);
        //Step 2: Find the distance between the points
        float temp = Mathf.Pow((end.localPosition.x - start.localPosition.x), 2) + Mathf.Pow((end.localPosition.y - start.localPosition.y), 2) + Mathf.Pow((end.localPosition.z - start.localPosition.z), 2);
        double distance = Mathf.Pow(temp, (float)0.5);
        //Find number of units
        //TODO: Handle edge case of 0 units 
        int noOfUnits = (int)(distance / Unit);
        Debug.Log(noOfUnits);
        Debug.Log(distance);

        //Find the parameter of the equation of the line
        Vector3 T = end.transform.localPosition - start.transform.localPosition;

        //Draw game objects at points of same unit length away.
        for (int i = 1; i <= noOfUnits; i++)
        {
            Vector3 Pos = start.transform.localPosition + i * ((float)Unit / (float)distance) * (T);
            Debug.Log(Pos);
            GameObject Wpoint = Instantiate(Waypoint, this.transform);
            Wpoint.transform.localPosition = Pos;
            Waypoints.AddLast(Wpoint);
            Debug.Log(i);

        }
        //Attach colliders to these game objects
        line = GetComponent<LineRenderer>();
        line.positionCount = noOfUnits;
        //Iterate through the list of Waypoints and add colliders
        foreach (GameObject Waypoint in Waypoints)
        {
            capsule = Waypoint.AddComponent<CapsuleCollider>();
            capsule.radius = (float)2.5;
            capsule.center = Vector3.zero;
            capsule.direction = 2; // z-axis for easier "lookat" orientation
            CollisionDetection CollisionDet = Waypoint.AddComponent<CollisionDetection>();
            CollisionDet.EntryColor = this.MouseOverColor;
            CollisionDet.ExitColor = this.WaypointColor;

            Waypoint.GetComponent<SphereCollider>().isTrigger = false;
            Waypoint.GetComponent<CapsuleCollider>().isTrigger = true;


        }

    }

    void Update()
    {
        int j = 0;
        foreach (GameObject Waypoint in Waypoints)
        {
            line.SetPosition(j++, Waypoint.transform.position);
            //capsule.transform.position = Waypoint.transform.position;
            //capsule.transform.LookAt(Waypoint.transform.position);
            //capsule.height = 5;
        }
        //line.SetPosition(0, start.position);
        //line.SetPosition(1, target.position);

        //capsule.transform.position = start.position + (target.position - start.position) / 2;
        //capsule.transform.LookAt(start.position);
        //capsule.height = (target.position - start.position).magnitude;
    }
    */

    /*
    Vector3 origin = new Vector3(0, 0, 0);
    Vector3 destination = new Vector3(10, 0, 0);
    public Vector3[] positions = new Vector3[2];
    LineRenderer lineRenderer;
    Color OriginalColor = Color.green;
    Color MouseOverColor = Color.red;
    public Material Orange;

    // Start is called before the first frame update
    void Start()
    {
        //Set origin and destination
        positions[0] = origin;
        positions[1] = destination;

        //Draw line
        lineRenderer = gameObject.AddComponent<LineRenderer>();
        lineRenderer.SetPositions(positions);
        lineRenderer.material.color = OriginalColor;

        //Attach an empty collider to Line Renderer
        //Step 1: Make an empty child game object of the line object.
        //Step 2: Add Box Collider Component to this Empty Child object.
        BoxCollider col = new GameObject().AddComponent<BoxCollider>();
        col.transform.parent = lineRenderer.transform;
        //Step 3: Set its size according to the line's width and height.
        float lineLength = Vector3.Distance(origin, destination);
        col.size = new Vector3(lineLength, 0.1f, 0.1f);
        //Step 4: Set its position according to line's position elements.
        Vector3 midpoint = (origin + destination) / 2;
        col.transform.position = midpoint;
        //Step 5: Calculate the angle between first element of line's position and secon element of the line's position 
        float angle = (Mathf.Abs(origin.y - destination.y) / Mathf.Abs(origin.x - destination.x));
        //and set Z coordinate of eulerAbgles of empty child object to this calculated angle.
        if ((origin.y < destination.y && origin.x > destination.x) || (destination.y < origin.y && destination.x > origin.x))
        {
            angle *= -1;
        }
        angle = Mathf.Rad2Deg * Mathf.Atan(angle);
        col.transform.Rotate(0, 0, angle);
        // col.material = (PhysicMaterial)Orange;
        Debug.Log("This prints");

    }

    void OnMouseOver()
    {
        Debug.Log("Here");
        // Change the color of the GameObject to red when the mouse is over GameObject
        lineRenderer.material.color = MouseOverColor;
    }

    void OnMouseExit()
    {
        // Reset the color of the GameObject back to normal
        lineRenderer.material.color = OriginalColor;
    }

    // Update is called once per frame
    void Update()
    {

    }
    */

    /*
    LineRenderer line;
    public Material EntryColor;
    public Material ExitColor;
    public Vector3 curPos;
    // Start is called before the first frame update
    void Start()
    {

    }

    // Update is called once per frame
    void Update()
    {

    }

    void OnMouseDown()
    {
        Vector3 mPos = Input.mousePosition;
        //mPos = Camera.main.ScreenToWorldPoint(mPos);
        this.GetComponentInParent<MeshRenderer>().material = EntryColor;
        curPos = this.GetComponentInParent<RectTransform>().position;
        //this.GetComponentInParent<RectTransform>().position = mPos;
        if (mPos.x > curPos.x)
        {
            Debug.Log("Mouse Position : " + mPos);
            Debug.Log("Curr Position :" + curPos);
            curPos.x++;
            this.GetComponentInParent<RectTransform>().position = curPos;
        }
        if (mPos.x < curPos.x)
        {
            Debug.Log("Mouse Position : " + mPos);
            Debug.Log("Curr Position :" + curPos);
            curPos.x--;
            this.GetComponentInParent<RectTransform>().position = curPos;
        }
        if (mPos.y > curPos.y)
        {
            Debug.Log("Mouse Position : " + mPos);
            Debug.Log("Curr Position :" + curPos);
            curPos.y++;
            this.GetComponentInParent<RectTransform>().position = curPos;
        }
        if (mPos.y < curPos.y)
        {
            Debug.Log("Mouse Position : " + mPos);
            Debug.Log("Curr Position :" + curPos);
            curPos.y--;
            this.GetComponentInParent<RectTransform>().position = curPos;
        }
        if (mPos.z > curPos.z)
        {
            Debug.Log("Mouse Position : " + mPos);
            Debug.Log("Curr Position :" + curPos);
            curPos.z++;
            this.GetComponentInParent<RectTransform>().position = curPos;
        }
        if (mPos.z < curPos.z)
        {
            Debug.Log("Mouse Position : " + mPos);
            Debug.Log("Curr Position :" + curPos);
            curPos.z--;
            this.GetComponentInParent<RectTransform>().position = curPos;
        }
    }
    void OnMouseEnter()
    {
        //Debug.Log("Here");
                Vector3 mPos = Input.mousePosition;
                Debug.Log(mPos);
                this.GetComponentInParent<MeshRenderer>().material = EntryColor;
                curPos = this.GetComponentInParent<RectTransform>().position;
                curPos.x++;
                this.GetComponentInParent<RectTransform>().position = curPos;
        //Vector3 pz = Camera.main.ScreenToWorldPoint(Input.mousePosition);
        //pz.z = 0;
        //gameObject.transform.position = pz;
        //// Change the color of the GameObject to red when the mouse is over GameObject
        //line.material = MouseOverColor;
    }

    void OnMouseExit()
    {
        this.GetComponentInParent<MeshRenderer>().material = ExitColor;
        //curPos.x--;
        //this.GetComponentInParent<RectTransform>().position = curPos;
        Debug.Log("Exiting");
        // Reset the color of the GameObject back to normal
        //line.material = OriginalColor;
    }
    */
}
