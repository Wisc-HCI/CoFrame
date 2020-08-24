using System;
using System.Collections;
using System.Collections.Generic;
using System.ComponentModel.Design;
using System.Diagnostics;
using UnityEngine;

public class HandMenuToolsScript : MonoBehaviour
{
    /*
     * Profile Type
     */

    [System.Serializable]
    public class LayoutProfile
    {
        public string name;
        public Vector3 StandardToolsPosition;
        public bool StandardToolsVisible;
        public Vector3 SetupToolsPosition;
        public bool SetupToolsVisible;
        public Vector3 TrajectoryToolsPosition;
        public bool TrajectoryToolsVisible;
        public bool DisplayCalibrateButton;
        public Vector3 SetupTrajectoryPosition;
    }

    /*
     * Attributes
     */

    public CustomHoloButtonCollectionScript setupTools = null;
    public CustomHoloButtonCollectionScript trajectoryTools = null;
    public CustomHoloButtonCollectionScript standardTools = null;
    public CustomHoloButtonCollectionScript setupTrajectoryTools = null;
    public StandardToolsLayoutScript standardToolsLayout = null;

    public EnvironmentScript environment = null;
    public CalibrationScript calibration = null;

    [SerializeField]
    private bool _displaySetupTrajectoryButtons = false;

    [SerializeField]
    private string displayMode = "hand";

    [SerializeField]
    private string state = "start";

    [SerializeField]
    private string activeTool = "select";

    [SerializeField]
    private bool editZoneState = false;

    [SerializeField]
    private int _layoutProfileSelected = 0;

    [SerializeField]
    private LayoutProfile[] handLayoutProfiles = new LayoutProfile[0];

    [SerializeField]
    private LayoutProfile[] tabletLayoutProfiles = new LayoutProfile[0];

    private Dictionary<string, CustomHoloButtonScript> buttons = new Dictionary<string, CustomHoloButtonScript>();

    /*
     * Display Mode Methods
     */

    public void SetDisplayModeToHand()
    {
        displayMode = "hand";
        UpdateLayout();
    }

    public void SetDisplayModeToTablet()
    {
        displayMode = "tablet";
        UpdateLayout();
    }

    public string GetDisplayMode()
    {
        return displayMode;
    }

    public void DisplaySetupTrajectoryButtons(bool value)
    {
        _displaySetupTrajectoryButtons = value;
        UpdateLayout();
    }

    /*
     * High-level state-machine control
     */

    private void Start()
    {
        // Collect buttons
        foreach (var b in setupTools.buttons)
        {
            b.callback = OnButtonPress;
            buttons.Add(b.identifier, b);
        }

        foreach (var b in trajectoryTools.buttons)
        {
            b.callback = OnButtonPress;
            buttons.Add(b.identifier, b);
        }

        foreach (var b in standardTools.buttons)
        {
            b.callback = OnButtonPress;
            buttons.Add(b.identifier, b);
        }

        foreach (var b in setupTrajectoryTools.buttons)
        {
            b.callback = OnButtonPress;
            buttons.Add(b.identifier, b);
        }

        // Set button states
        SetBackgroundVisible(activeTool, true);
        SetBackgroundVisible("edit-zone", editZoneState);

        // Update with current profile
        UpdateLayout();
    }

    private void OnButtonPress(string identifier)
    {
        switch (identifier)
        {
            case "select":
                _SelectTransition();
                break;
            case "calibrate":
                _CalibrateTransition();
                break;

            case "add-location-spawn":
                _AddLocationTransition("spawn");
                break;
            case "add-location-robot":
                _AddLocationTransition("robot");
                break;
            case "delete-location":
                _DeleteLocationTransition();
                break;
            case "edit-zone":
                _EditZoneTransition();
                break;

            case "create-trajectory":
                _CreateTrajectoryTransition();
                break;
            case "delete-trajectory":
                _DeleteTrajectoryTransition();
                break;

            case "add-waypoint-spawn":
                _AddWaypointTransition("spawn");
                break;
            case "add-waypoint-robot":
                _AddWaypointTransition("robot");
                break;
            case "delete-waypoint":
                _DeleteWaypointTransition();
                break;
            case "reorder-waypoints":
                _ReorderWaypointsTransition();
                break;
            case "render-trajectory":
                _RenderTrajectoryTransition();
                break;
            case "done":
                _DoneTransition();
                break;  
        }
    }

    private void OnCalibrationComplete(bool success)
    {
        SetBackgroundVisible(activeTool, false);

        activeTool = "calibrate-finished";

        if (state == "start")
        {
            if (success)
            {
                SetupStateTransition();
            }
            else
            {
                _SelectTransition();
            }
            
        }
        else if (state == "setup")
        {
            _SelectTransition();
        }
    }

    private void OnReorderWaypointsComplete(bool success)
    {
        _SelectTransition();
    }

    private void OnRenderTrajectoryComplete(bool success)
    {
        _SelectTransition();
    }

    private void OnCreateTrajectoryComplete(bool success)
    {
        if (success)
        {
            TrajectoryStateTransition();
        }
    }

    /*
     * State Transitions
     */

    private void StartStateTransition()
    {
        bool process = false;

        if (state  == "setup")
        {
            process = true;
            if (editZoneState)
            {
                editZoneState = false;
                UpdateEditZone();
            }
        }
        else if (state == "trajectory")
        {
            process = true;
        }

        if (process)
        {
            _SelectTransition();
            state = "start";

            _layoutProfileSelected = 0;
            UpdateLayout();
        }
    }

    private void SetupStateTransition()
    {
        bool process = false;

        if (state == "start")
        {
            process = true;
        }
        else if (state == "trajectory")
        {
            process = true;
        }

        if (process)
        {
            _SelectTransition();
            state = "setup";

            _layoutProfileSelected = 1;
            UpdateLayout();
        }
    }

    private void TrajectoryStateTransition()
    {
        bool process = false;

        if (state == "start")
        {
            process = true;
        }
        else if (state == "setup")
        {
            process = true;
            if (editZoneState)
            {
                editZoneState = false;
                UpdateEditZone();
            }
        }

        if (process)
        {
            state = "trajectory";

            _layoutProfileSelected = 2;
            UpdateLayout();

            _SelectTransition();
        }
    }

    /*
     * Active Tool Transitions
     */

    private void _SelectTransition()
    {
        if (activeTool == "calibrate")
        {
            calibration.Stop();
        }
        else if (activeTool == "render-trajectory")
        {
            environment.StopRenderingTrajectory();
        }
        else if (activeTool == "create-trajectory")
        {
            environment.StopCreatingTrajectory();
        }
        else if (activeTool == "reorder-waypoints")
        {
            environment.StopReorderingWaypoints();
        }

        SetBackgroundVisible(activeTool, false);
        activeTool = "select";
        SetBackgroundVisible(activeTool, true);

        environment.EnableSelectTool();
    }

    private void _CalibrateTransition()
    {
        bool perform = false;

        if (state == "start")
        {
            if (activeTool == "select")
            {
                perform = true;
            }
        }
        else if (state == "setup")
        {
            if (activeTool == "select")
            {
                perform = true;
            }
            else if (activeTool == "delete-location")
            {
                perform = true;
            }
        }
        else if (state == "trajectory")
        {
            if (activeTool == "select")
            {
                perform = true;
            }
            else if (activeTool == "delete-waypoint")
            {
                perform = true;
            }
            else if (activeTool == "reorder-waypoints")
            {
                perform = true;
            }
        }

        if (perform)
        {
            SetBackgroundVisible(activeTool, false);
            activeTool = "calibrate";
            SetBackgroundVisible("calibrate", true);

            environment.DisableTools();

            calibration.Calibrate(OnCalibrationComplete);
        }
    }

    private void _AddLocationTransition(string place)
    {
        if (state == "setup" && activeTool == "select")
        {
            if (place == "spawn")
            {
                environment.CreateLocationAtWorldSpawn();
            }
            else if (place == "robot")
            {
                environment.CreateLocationAtRobotSpawn();
            }
        }
        else if (state == "setup" && activeTool == "delete-location")
        {
            if (place == "spawn")
            {
                environment.CreateLocationAtWorldSpawn();
            }
            else if (place == "robot")
            {
                environment.CreateLocationAtRobotSpawn();
            }

            // enable select options
            _SelectTransition();
        }
    }

    private void _DeleteLocationTransition()
    {
        var process = false;

        if (state == "setup")
        {
            if (activeTool == "create-trajectory")
            {
                environment.StopCreatingTrajectory();
            }
            process = true;
        }

        if (process)
        {
            SetBackgroundVisible(activeTool, false);
            activeTool = "delete-location";
            SetBackgroundVisible(activeTool, true);

            environment.EnableDeleteLocationTool();
        }
    }

    private void _EditZoneTransition()
    {
        if (state == "setup")
        {
            // Toggle the Zone state
            editZoneState = !editZoneState;
            UpdateEditZone();
        }
    }


    private void _CreateTrajectoryTransition()
    {
        var process = false;

        if (state  == "setup")
        {
            process = true;
        }

        if (process)
        {
            SetBackgroundVisible(activeTool, false);
            activeTool = "create-trajectory";
            SetBackgroundVisible(activeTool, true);

            environment.CreateTrajectoryTool(OnCreateTrajectoryComplete);
        }
    }

    private void _DeleteTrajectoryTransition()
    {
        var process = false;

        if (state == "setup")
        {
            if (activeTool == "create-trajectory")
            {
                environment.StopCreatingTrajectory();
            }
            process = true;
        }

        if (process)
        {
            SetBackgroundVisible(activeTool, false);
            activeTool = "delete-trajectory";
            SetBackgroundVisible(activeTool, true);

            environment.EnableDeleteTrajectoryTool();
        }
    }


    private void _AddWaypointTransition(string place)
    {
        if (state == "trajectory" && activeTool == "select")
        {
            if (place == "spawn")
            {
                environment.CreateWaypointAtWorldSpawn();
            }
            else if (place == "robot")
            {
                environment.CreateWaypointAtRobotSpawn();
            }
        }
        else if (state == "trajectory" && activeTool == "reorder-waypoints")
        {
            if (place == "spawn")
            {
                environment.CreateWaypointAtWorldSpawn();
            }
            else if (place == "robot")
            {
                environment.CreateWaypointAtRobotSpawn();
            }

            // enable select options
            _SelectTransition();
        }
        else if (state == "trajectory" && activeTool == "delete-waypoint")
        {
            if (place == "spawn")
            {
                environment.CreateWaypointAtWorldSpawn();
            }
            else if (place == "robot")
            {
                environment.CreateWaypointAtRobotSpawn();
            }

            // enable select options
            _SelectTransition();
        }
    }
    
    private void _DeleteWaypointTransition()
    {
        var process = false;

        if (state == "trajectory")
        {
            if (activeTool == "reorder-waypoints")
            {
                environment.StopReorderingWaypoints();
            }
            else if (activeTool == "render-trajectory")
            {
                environment.StopRenderingTrajectory();
            }
            process = true;
        }

        if (process)
        {
            SetBackgroundVisible(activeTool, false);
            activeTool = "delete-waypoint";
            SetBackgroundVisible(activeTool, true);

            environment.EnableDeleteWaypointTool();
        }
    }

    private void _ReorderWaypointsTransition()
    {
        var process = false;

        if (state == "trajectory")
        {
            if (activeTool == "render-trajectory")
            {
                environment.StopRenderingTrajectory();
            }
            process = true;
        }

        if (process)
        {
            SetBackgroundVisible(activeTool, false);
            activeTool = "reorder-waypoints";
            SetBackgroundVisible(activeTool, true);

            environment.ReorderWaypointsTool(OnReorderWaypointsComplete);
        }
    }

    private void _RenderTrajectoryTransition()
    {
        var process = false;

        if (state == "trajectory")
        {
            if (activeTool == "reorder-waypoints")
            {
                environment.StopReorderingWaypoints();
            }
            process = true;
        }

        if (process)
        {
            SetBackgroundVisible(activeTool, false);
            activeTool = "render-trajectory";
            SetBackgroundVisible(activeTool, true);

            environment.RenderTrajectoryTool(OnRenderTrajectoryComplete);
        }
    }

    private void _DoneTransition()
    {
        SetupStateTransition();
    }

    /*
     * Utility Methods
     */

    private void UpdateLayout()
    {
        // get profile set to display
        LayoutProfile[] profiles = null;
        if (displayMode == "hand")
        {
            profiles = handLayoutProfiles;
        }
        else if (displayMode == "tablet")
        {
            profiles = tabletLayoutProfiles;
        }

        // get active profile
        var profile = profiles[_layoutProfileSelected];

        // update UI to conform to profile
        standardTools.transform.localPosition = profile.StandardToolsPosition;
        standardTools.gameObject.SetActive(profile.StandardToolsVisible);

        setupTools.transform.localPosition = profile.SetupToolsPosition;
        setupTools.gameObject.SetActive(profile.SetupToolsVisible);

        trajectoryTools.transform.localPosition = profile.TrajectoryToolsPosition;
        trajectoryTools.gameObject.SetActive(profile.TrajectoryToolsVisible);

        setupTrajectoryTools.transform.localPosition = profile.SetupTrajectoryPosition;
        setupTrajectoryTools.gameObject.SetActive(state == "setup" && _displaySetupTrajectoryButtons);

        // visible calibrate
        standardToolsLayout.displayButtons = profile.DisplayCalibrateButton;
    }
    
    private void UpdateEditZone()
    {
        // Button State
        SetBackgroundVisible("edit-zone", editZoneState);

        // Command expert controls to make visible if not
        if (editZoneState)
        {
            environment.SetOccupancyZonesVisible(true);
        }

        // Toggle the zone controls
        environment.SetOccupancyZoneControlsVisible(editZoneState);
    }

    private void SetBackgroundVisible(string id, bool value)
    {
        if (buttons.ContainsKey(id))
        {
            buttons[id].backgroundVisible = value;
        }
    }
}
