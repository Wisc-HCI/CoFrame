using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using Cobots;


public class TrajectoryManagerTestScript : MonoBehaviour
{
    public Toggle Step1;
    public Toggle Step2;
    public Toggle Step3;
    public Toggle Step4;

    public GameObject DisableStep1;
    public GameObject DisableStep2;
    public GameObject DisableStep3;
    public GameObject DisableStep3Field;
    public GameObject DisableStep4;

    public InputField Step3Field;

    public InteractiveMarkerManagerScript markerManager;
    public TrajectoryManagerScript trajectoryManager;
    public TraceManagerScript traceManager;

    private int step = 0;

    private Location startLoc = null;
    private Location endLoc = null;
    private Trajectory trajectory = null;
    private Trace trace = null;

    public Transform spawnPoint;

    private void Start()
    {
        DisplayStep();
        markerManager.markerSelectedCallback = OnMarkerSelected;
        trajectoryManager.colliderClickCallback = OnTrajectoryColliderClicked;
        traceManager.colliderClickCallback = OnTraceColliderClicked;
    }

    public void OnStep1Pressed() // START LOCATION
    {
        step = 1;
        DisplayStep();

        // generate start location
        startLoc = new Location(Position.FromUnity(spawnPoint.localPosition), Orientation.FromUnity(spawnPoint.localRotation));
        markerManager.OnWaypointAdd(startLoc);
        markerManager.SetVisibile(startLoc.uuid, true);
    }

    public void OnStep2Pressed() // END LOCATION
    {
        step = 2;
        DisplayStep();

        // Generate end location
        endLoc = new Location(Position.FromUnity(spawnPoint.localPosition), Orientation.FromUnity(spawnPoint.localRotation));
        markerManager.OnWaypointAdd(endLoc);
        markerManager.SetVisibile(endLoc.uuid, true);

        // Create trajectory
        trajectory = new Trajectory(startLoc.uuid, endLoc.uuid);
        trajectoryManager.OnTrajectoryCreate(trajectory);
        trajectoryManager.GetTrajectory(trajectory.uuid).visible = true;
    }

    public void OnStep3Pressed() // ADD WAYPOINTS
    {
        step = 3;
        DisplayStep();

        // generate inputField number of waypoints to trajectory
        var str = Step3Field.text;
        if (str == "")
        {
            str = "0";
        }

        var num = int.Parse(str);
        for (int i=0; i<num; i++)
        {
            var wp = new Waypoint(Position.FromUnity(spawnPoint.localPosition), Orientation.FromUnity(spawnPoint.localRotation));
            trajectory.AddWaypoint(wp);
            markerManager.OnWaypointAdd(wp);
            trajectoryManager.OnAddWaypoint(trajectory, wp.uuid);
        }
    }

    public void OnStep4Pressed() // RENDER
    {
        step = 4;
        DisplayStep();

        // generate trace
        List<TraceDataPoint> eeTrace = new List<TraceDataPoint>();
        var markers = new List<Waypoint>();
        markers.Add(startLoc);
        markers.AddRange(trajectory.waypoints);
        markers.Add(endLoc);

        const float TIME = 10;
        for (int i=1; i<markers.Count; i++)
        {
            var msPos = markers[i-1].position.ToUnity();
            var mePos = markers[i].position.ToUnity();
            var msRot = markers[i - 1].orientation.ToUnity();
            var meRot = markers[i].orientation.ToUnity();

            for (int dt=0; dt < (int)(TIME); dt++) {
                var off = new Vector3(Random.Range(-0.005f, 0.005f), Random.Range(-0.005f, 0.005f), Random.Range(-0.005f, 0.005f));
                var pos = Vector3.Lerp(msPos, mePos, dt / TIME) + off;
                var rot = Quaternion.Lerp(msRot, meRot, dt / TIME);

                eeTrace.Add(new TraceDataPoint(Position.FromUnity(pos), Orientation.FromUnity(rot), 0));
            }

            eeTrace.Add(new TraceDataPoint(Position.FromUnity(mePos), Orientation.FromUnity(meRot), 0));

        }

        // display trace
        Dictionary<string, List<TraceDataPoint>> data = new Dictionary<string, List<TraceDataPoint>>();
        data.Add("ee", eeTrace);
        trace = new Trace("ee", data);
        traceManager.OnTraceAdd(trace);
        traceManager.GetTrace(trace.uuid).visible = true;
        traceManager.GetTrace(trace.uuid).SetEndEffectorPathDrawMode(TraceRendererScript.DrawMode.ColorCoded);

        // hide trajectory
        trajectoryManager.GetTrajectory(trajectory.uuid).visible = false;
    }

    public void OnResetPressed() // RESET
    {
        step = 0;
        DisplayStep();

        // delete all locations, waypoints, trajectories, and traces
        if (trajectory != null)
        {
            foreach (var w in trajectory.waypoints)
            {
                trajectoryManager.OnDeleteWaypoint(trajectory, w.uuid);
                markerManager.OnWaypointDelete(w.uuid);
            }
            trajectoryManager.OnTrajectoryDelete(trajectory.uuid);
        }

        if (startLoc != null)
        {
            markerManager.OnWaypointDelete(startLoc.uuid);
        }
        
        if (endLoc != null)
        {
            markerManager.OnWaypointDelete(endLoc.uuid);
        }
        
        if (trace != null)
        {
            traceManager.OnTraceDelete(trace.uuid);
        }
        
        startLoc = null;
        endLoc = null;
        trajectory = null;
        trace = null;
    }

    private void DisplayStep()
    {
        DisableStep1.SetActive(step != 0);
        DisableStep2.SetActive(step != 1);
        DisableStep3.SetActive(step != 2);
        DisableStep3Field.SetActive(step != 2);
        DisableStep4.SetActive(step != 3);

        Step1.isOn = step >= 1;
        Step2.isOn = step >= 2;
        Step3.isOn = step >= 3;
        Step4.isOn = step >= 4;
    }

    private void OnMarkerSelected(AbstractMarker marker, bool state)
    {
        // TODO
    }

    private void OnTrajectoryColliderClicked(string uuid, Vector3 position, Quaternion orientation, int idx)
    {
        var wp = new Waypoint(Position.FromUnity(position), Orientation.FromUnity(orientation));
        trajectory.InsertWaypoint(idx, wp);
        markerManager.OnWaypointAdd(wp);
        trajectoryManager.OnAddWaypoint(trajectory, wp.uuid);
    }

    private void OnTraceColliderClicked(string id, Vector3 position, Quaternion orientation, int idx)
    {
        //TODO
    }
}
