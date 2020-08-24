using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class TrajectoryTestScript : MonoBehaviour
{
    public TrajectoryRenderScript Trajectory;

    public TrajectoryPointScript StartLocation;
    public TrajectoryPointScript EndLocation;
    public TrajectoryPointScript WaypointOne;
    public TrajectoryPointScript WaypointTwo;

    public GameObject DisableCreateTrajPanel;
    public GameObject DisableDeleteTrajPanel;
    public GameObject DisableAddWaypointPanel;
    public GameObject DisableDeleteWaypointPanel;

    public int INDEX = 1;

    private bool createdTraj = false;
    private bool addedWaypoint = false;

    private Dictionary<string, bool> selectedMarkers = new Dictionary<string, bool>();

    private void Awake()
    {
        SetDisablePanelState();

        StartLocation.GetComponent<Cobots.AbstractMarker>().selectedCallback = OnMarkerSelected;
        EndLocation.GetComponent<Cobots.AbstractMarker>().selectedCallback = OnMarkerSelected;
        WaypointOne.GetComponent<Cobots.AbstractMarker>().selectedCallback = OnMarkerSelected;
        WaypointTwo.GetComponent<Cobots.AbstractMarker>().selectedCallback = OnMarkerSelected;
    }

    private void Start()
    {
        selectedMarkers.Add(StartLocation.uuid, false);
        selectedMarkers.Add(EndLocation.uuid, false);
        selectedMarkers.Add(WaypointOne.uuid, false);
        selectedMarkers.Add(WaypointTwo.uuid, false);
    }

    private void Update()
    {
        foreach (KeyValuePair<string, bool> entry in selectedMarkers)
        {
            if (entry.Value)
            {
                Trajectory.OnManipulatedWaypoint(entry.Key);
            }
        }
    }

    public void OnCreateTrajectory()
    {
        createdTraj = true;
        SetDisablePanelState();

        var w = new List<TrajectoryPointScript>();
        w.Add(WaypointOne);
        Trajectory.OnCreate(StartLocation, EndLocation, w);
    }

    public void OnDeleteTrajectory()
    {
        createdTraj = false;
        addedWaypoint = false;
        SetDisablePanelState();

        Trajectory.OnDelete();
    }

    public void OnAddWaypoint()
    {
        addedWaypoint = true;
        SetDisablePanelState();

        Trajectory.OnAddWaypoint(WaypointTwo, INDEX);
    }

    public void OnDeleteWaypoint()
    {
        addedWaypoint = false;
        SetDisablePanelState();

        Trajectory.OnDeleteWaypoint(WaypointTwo.uuid);
    }

    private void OnMarkerSelected(Cobots.AbstractMarker marker, bool state)
    {
        selectedMarkers[marker.uuid] = state;
    }

    private void SetDisablePanelState()
    {
        DisableCreateTrajPanel.SetActive(createdTraj);
        DisableDeleteTrajPanel.SetActive(!createdTraj);
        DisableAddWaypointPanel.SetActive(!createdTraj || addedWaypoint);
        DisableDeleteWaypointPanel.SetActive(!createdTraj || !addedWaypoint);
    }
}
