using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Cobots;
using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.Messages.Standard;
using Microsoft.MixedReality.Toolkit.UI;

//TODO handle locations
//TODO handle waypoints
//TODO handle robot markers

//TODO handle trajectories
//TODO handle traces

public class EnvironmentScript : MonoBehaviour
{
    public float DEFAULT_FLOOR_HEIGHT = -0.5f;

    public ReachSphereScript reachSphere = null;
    public RobotMarkerManagerScript robotMarkers = null;
    public CollisionMeshManagerScript collisionMeshes = null;
    public PinchPointManagerScript pinchPoints = null;
    public TrajectoryManagerScript trajectories = null;
    public TraceManagerScript traces = null;
    public SpawnManagerScript spawnTargets = null;
    public OccupancyZoneManagerScript occupancyZones = null;
    public InteractiveMarkerManagerScript markers = null;

    public Environment environmentModel = null;
    private AttributeTraceProcessor traceProcessor = null;

    public Microsoft.MixedReality.Toolkit.UI.ProgressIndicatorOrbsRotator progress = null;

    public string activeTrajectoryUuid { get; private set; } = null; 

    private void Start()
    {
        // Default Reach Sphere
        var reachSphereModel = new ReachSphere();

        // Default Pinch Points
        var pinchPointsModel = new List<PinchPoint>();
        foreach (var p in pinchPoints.pinchPoints)
        {
            pinchPointsModel.Add(new PinchPoint(name: p.name));
        }

        // Default Collision Meshes
        var collisionMeshesModel = new List<CollisionMesh>();
        foreach (var c in collisionMeshes.collisionMeshes)
        {
            collisionMeshesModel.Add(new CollisionMesh(name: c.name));
        }

        // Default Occupancy Zones
        var occupancyZonesModel = new List<OccupancyZone>();
        var humanZone = new OccupancyZone(occupancy_type: "human", name: "human", posZ: -0.5f);
        occupancyZonesModel.Add(humanZone);
        occupancyZones.HumanOccupancyZone.model = humanZone;

        var robotZone = new OccupancyZone(occupancy_type: "robot", name: "robot");
        occupancyZonesModel.Add(robotZone);
        occupancyZones.RobotOccupancyZone.model = robotZone;

        SetOccupancyZoneFloorHeight(DEFAULT_FLOOR_HEIGHT);

        // Default Location List
        var locationsModel = new List<Location>();

        // Default Trajectory List
        var trajectoriesModel = new List<Trajectory>();

        // Construct model
        environmentModel = new Environment(
            reachSphere: reachSphereModel,
            pinchPoints: pinchPointsModel,
            collisionMeshes: collisionMeshesModel,
            occupancyZones: occupancyZonesModel,
            locations: locationsModel,
            trajectories: trajectoriesModel
        );

        // Construct the trace processor
        traceProcessor = new AttributeTraceProcessor();
        traceProcessor.SubscribeToType("node.reach-sphere.", ReachSphereCallback);
        traceProcessor.SubscribeToType("node.pinch-point.", PinchPointCallback);
        traceProcessor.SubscribeToType("node.collision-mesh.", CollisionMeshCallback);
        traceProcessor.SubscribeToType("node.occupancy-zone.", OccupancyZoneCallback);
        traceProcessor.SubscribeToType("node.environment.", EnvironmentCallback);
        traceProcessor.SubscribeToType("node.pose.waypoint.location.", LocationAndWaypointCallback);
        traceProcessor.SubscribeToType("node.pose.waypoint.", LocationAndWaypointCallback);

        environmentModel.changesCallback = traceProcessor.ProcessTrace;

        // Need to perform a deep update
        environmentModel.DeepUpdate();
    }

    public void CalibratePositionAt(Transform target) // TODO
    {
        transform.position = target.position;
        transform.rotation = target.rotation;
    }

    private void EnvironmentCallback(Dictionary<string, string> entry)
    {
        if (entry["verb"] == "set")
        {
            if (entry["attribute"] == "reach_sphere")
            {
                SetReachSphereState();
            }
            else if (entry["attribute"] == "collision_meshes")
            {
                foreach (var c in environmentModel.collisionMeshes)
                {
                    collisionMeshes.OnCollisionMeshUpdate(c);
                }
            }
            else if (entry["attribute"] == "pinch_points")
            {
                foreach (var p in environmentModel.pinchPoints)
                {
                    pinchPoints.OnPinchPointUpdate(p);
                }
            }
            else if (entry["attribute"] == "occupancy_zones")
            {
                foreach (var o in environmentModel.occupancyZones)
                {
                    occupancyZones.OnUpdate(o);
                }
            }
            else if (entry["attribute"] == "locations")
            {
                // TODO
            }
            else if (entry["attribute"] == "trajectories")
            {

            }
        }
        else if (entry["verb"] == "add")
        {
            if (entry["attribute"] == "locations")
            {
                // TODO
            }
            else if (entry["attribute"] == "trajectories")
            {
                // TODO
            }
        }
        else if (entry["verb"] == "delete")
        {
            if (entry["attribute"] == "locations")
            {
                // TODO
            }
            else if (entry["attribute"] == "trajectories")
            {
                // TODO
            }
        }
    }

    /*
     * Tools
     */

    public void EnableDeleteLocationTool() // TODO
    {
        // TODO
    }

    public void EnableDeleteWaypointTool() // TODO
    {
        // TODO
    }

    public void EnableDeleteTrajectoryTool() // TODO
    {
        // TODO
    }

    public void EnableSelectTool() // TODO
    {
        // TODO
    }

    public void DisableTools() // TODO
    {
        // TODO
    }

    public void RenderTrajectoryTool(System.Action<bool> callback) // TODO
    {
        callback?.Invoke(true); // TODO
    }

    public void StopRenderingTrajectory() // TODO
    {
        // TODO
    }

    public void CreateTrajectoryTool(System.Action<bool> callback) // TODO
    {
        callback?.Invoke(true); // TODO
    }

    public void StopCreatingTrajectory() // TODO
    {
        // TODO
    }

    public void ReorderWaypointsTool(System.Action<bool> callback) // TODO
    {
        callback?.Invoke(true); // TODO
    }

    public void StopReorderingWaypoints() // TODO
    {
        // TODO
    }

    /*
     * Reach Sphere
     */

    public void SetReachSphereVisible(bool value)
    {
        reachSphere.expand = value;
    }

    public bool GetReachSphereVisible()
    {
        return reachSphere.expand;
    }

    private void SetReachSphereState()
    {
        var state = environmentModel.reachSphere.state;
        if (state == ReachSphere.GOOD_STATE)
        {
            reachSphere.colorState = ReachSphereScript.ColorState.Good;
        }
        else if (state == ReachSphere.WARN_STATE)
        {
            reachSphere.colorState = ReachSphereScript.ColorState.Warn;
        }
        else if (state == ReachSphere.ERROR_STATE)
        {
            reachSphere.colorState = ReachSphereScript.ColorState.Error;
        }
    }

    private void ReachSphereCallback(Dictionary<string, string> entry)
    {
        if ((entry["verb"] == "set" || entry["verb"] == "update") && entry["attribute"] == "state")
        {
            SetReachSphereState();
        }
    }

    /*
     * Pinch Points 
     */

    public void SetPinchPointsVisible(bool value)
    {
        foreach (var p in environmentModel.pinchPoints)
        {
            pinchPoints.SetVisible(p.name, value);
        }
    }

    public void SetPinchPointVisible(string uuid, bool value)
    {
        var name = environmentModel.pinchPoints.Find(x => x.uuid == uuid).name;
        pinchPoints.SetVisible(name, value);
    }

    public bool GetPinchPointVisible(string uuid)
    {
        var name = environmentModel.pinchPoints.Find(x => x.uuid == uuid).name;
        return pinchPoints.GetVisible(name);
    }

    private void PinchPointCallback(Dictionary<string,string> entry)
    {
        if ((entry["verb"] == "set" || entry["verb"] == "update") && entry["attribute"] == "state")
        {
            pinchPoints.OnPinchPointUpdate(environmentModel.pinchPoints.Find(x => x.uuid == entry["uuid"]));
        }
    }

    /*
     * Collision Meshes
     */

    public void SetCollisionMeshesVisible(bool value)
    {
        foreach (var c in environmentModel.collisionMeshes)
        {
            collisionMeshes.SetVisible(c.name, value);
        }
    }

    public void SetCollisionMeshVisible(string uuid, bool value)
    {
        var name = environmentModel.collisionMeshes.Find(x => x.uuid == uuid).name;
        collisionMeshes.SetVisible(name, value);
    }

    public bool GetCollisionMeshVisible(string uuid)
    {
        var name = environmentModel.collisionMeshes.Find(x => x.uuid == uuid).name;
        return collisionMeshes.GetVisible(name);
    }

    private void CollisionMeshCallback(Dictionary<string, string> entry)
    {
        if ((entry["verb"] == "set" || entry["verb"] == "update") && entry["attribute"] == "state")
        {
            collisionMeshes.OnCollisionMeshUpdate(environmentModel.collisionMeshes.Find(x => x.uuid == entry["uuid"]));
        }
    }

    /*
     * Occupancy Zones
     */

    public void SetOccupancyZonesVisible(bool value)
    {
        occupancyZones.SetVisible(value);
    }

    public void SetOccupancyZoneVisible(string uuid, bool value)
    {
        occupancyZones.SetVisible(uuid, value);
    }

    public bool GetOccupancyZoneVisible(string uuid)
    {
        return occupancyZones.GetVisible(uuid);
    }

    public void SetOccupancyZoneControlsVisible(bool value)
    {
        occupancyZones.SetControlsVisible(value);
    }

    public void SetOccupancyZoneFloorHeight(float height)
    {
        occupancyZones.SetHeight(height);
    }

    private void OccupancyZoneCallback(Dictionary<string, string> entry)
    {
        if (entry["verb"] == "set" || entry["verb"] == "update")
        {
            occupancyZones.OnUpdate(environmentModel.occupancyZones.Find(x => x.uuid == entry["uuid"]));
        }
    }

    /*
     * Locations
     */

    public void CreateLocationAtWorldSpawn() // TODO
    {
        //spawnTargets.
        // TODO
    }

    public void CreateLocationAtRobotSpawn() // TODO
    {
        // TODO
    }

    private void LocationAndWaypointCallback(Dictionary<string, string> entry)
    {
        if (entry["verb"] == "set")
        {
            var w = (Waypoint)(environmentModel.cache.Get(entry["uuid"], "waypoint"));
            markers.OnWaypointUpdate(w);
            robotMarkers.OnWaypointUpdate(w);
        } 
        else if (entry["verb"] == "add")
        {
            var w = (Waypoint)(environmentModel.cache.Get(entry["uuid"], "waypoint"));
            markers.OnWaypointAdd(w);
            robotMarkers.OnWaypointAdd(w);
        }
        else if (entry["verb"] == "delete")
        {
            markers.OnWaypointDelete(entry["uuid"]);
            robotMarkers.OnWaypointDelete(entry["uuid"]);
        }
        else if (entry["verb"] == "update")
        {
            var w = (Waypoint)(environmentModel.cache.Get(entry["uuid"], "waypoint"));

            if (markers.GetMarker(entry["uuid"]) == null)
            {
                markers.OnWaypointAdd(w);
                robotMarkers.OnWaypointAdd(w);
            }
            else
            {
                markers.OnWaypointUpdate(w);
                robotMarkers.OnWaypointUpdate(w);
            }
        }
    }

    /*
     * Waypoints
     */

    public void CreateWaypointAtWorldSpawn() // TODO
    {
        // TODO
    }

    public void CreateWaypointAtRobotSpawn() // TODO
    {
        // TODO
    }

    /*
     * Robot Markers
     */


    /*
     * Trajectories
     */


    /*
     * Traces
     */

}




































//private RosSocket rosSocket;

/*
private void Start()
{
    rosSocket = GameObject.FindGameObjectWithTag("ROS Bridge").GetComponent<RosConnector>().RosSocket;

    // expert view options
    rosEndEffectorPathOptionSub = rosSocket.Subscribe<Bool>("environment/view_options/end_effector_path", EndEffectorPathOptionCallback);
    rosJointPathOptionSub = rosSocket.Subscribe<Bool>("environment/view_options/joint_path", JointPathOptionCallback);
    rosToolPathOptionSub = rosSocket.Subscribe<Bool>("environment/view_options/tool_path",ToolPathOptionCallback);
    rosRobotMarkersOptionSub = rosSocket.Subscribe<Bool>("environment/view_options/robot_markers",RobotMarkersOptionCallback);
    rosReachSphereOptionSub = rosSocket.Subscribe<Bool>("environment/view_options/reach_sphere",ReachSPhereOptionCallback);
    rosPayloadOptionSub = rosSocket.Subscribe<Bool>("environment/view_options/payload",PayloadOptionCallback);

    // subprogram to render

    // collisions to display
}

private void EndEffectorPathOptionCallback(Bool msg)
{
    //trajectories.RenderEndEffectorPaths(msg.data);
}

private void JointPathOptionCallback(Bool msg)
{
    //trajectories.RenderJointPaths(msg.data);
}

private void ToolPathOptionCallback(Bool msg)
{
    //trajectories.RenderToolPaths(msg.data);
}

private void RobotMarkersOptionCallback(Bool msg)
{
    //robotMarkers.RenderMarkers(msg.data);
}

private void ReachSPhereOptionCallback(Bool msg)
{
    reachSphere.expand = msg.data;
}

private void PayloadOptionCallback(Bool msg)
{
    // TODO
}

/*
public ApplicationDataScript app;

public GameObject LocationPrefab;
public GameObject WaypointPrefab;
public GameObject TrajectoryPrefab;

public RobotMarkerManagerScript robotMarkerManager;

private System.Action<string> locationCallback = null;
private System.Action<string> waypointCallback = null;
private List<LocationScript> locations = new List<LocationScript>();
private List<WaypointScript> waypoints = new List<WaypointScript>();
private List<TrajectoryRenderScript> lineTrajectories = new List<TrajectoryRenderScript>();
private List<TrajectoryRenderScript> traceTrajectories = new List<TrajectoryRenderScript>();

private string scene = null;
private string activeUuid = null;

//=========================================================================
//  Public Interface
//=========================================================================

public void Initialize(ApplicationDataScript app)
{
    this.app = app;
}

public void SetScene(string scene, string uuid)
{
    // tear down old scene
    ClearScene();

    // build up new scene
    this.scene = scene;
    activeUuid = uuid;
    switch (scene)
    {
        case "setup":
            RenderSetupScene();
            break;
        case "program":
            RenderProgramScene();
            break;
        case "operate":
            RenderOperateScene();
            break;
        case "move":
            RenderMoveScene();
            break;
        case "trajectory":
            RenderTrajectoryScene();
            break;
        case "grip":
            RenderGripScene();
            break;
        case "delay":
            RenderDelayScene();
            break;
        case "machine":
            RenderMachineScene();
            break;
        default:
            this.scene = null;
            break;
    }  
}

public void Refresh()
{
    SetScene(scene, activeUuid);
}

public Transform GetSceneFrame()
{
    return gameObject.transform;
}

public void StartSelectingLocations(System.Action<string> cb)
{
    locationCallback = cb;

    foreach (var loc in locations)
    {
        loc.Highlighted(true);
    }
}

public void StopSelectingLocations()
{
    locationCallback = null;

    foreach (var loc in locations)
    {
        loc.Highlighted(false);
    }
}

public void IndicateSelectedLocation(string uuid, bool val)
{
    int index = FindLocationById(uuid);
    locations[index].Selected(val);
}

//=========================================================================
//  Scene Renderer
//=========================================================================

private void ClearScene()
{
    foreach (var l in locations)
    {
        Destroy(l.gameObject);
    }
    locations.Clear();

    foreach (var w in waypoints)
    {
        Destroy(w.gameObject);
    }
    waypoints.Clear();

    foreach (var t in traceTrajectories)
    {
        Destroy(t.gameObject);
    }
    traceTrajectories.Clear();

    foreach (var t in lineTrajectories)
    {
        Destroy(t.gameObject);
    }
    lineTrajectories.Clear();

    robotMarkerManager.CreateMarkerVisualization(new List<double[]>());
}

private void RenderSetupScene()
{
    List<double[]> joints = new List<double[]>();

    // display all locations
    foreach (var loc in app.locations)
    {
        GameObject obj = Instantiate(LocationPrefab, GetSceneFrame(), false);
        obj.transform.Translate(loc.Value.position);
        obj.transform.Rotate(loc.Value.orientation.eulerAngles);

        LocationScript ls = obj.GetComponent<LocationScript>();
        ls.selectedCallback = LocationClicked;
        ls.Initialize(loc.Key, loc.Value.name, true);
        ls.Selected(false);

        locations.Add(ls);
        joints.Add(loc.Value.joints);
    }

    robotMarkerManager.CreateMarkerVisualization(joints);
}

private void RenderProgramScene()
{
    // capture relevant data from primitives
    List<double[]> joints = new List<double[]>();
    List<string> usedLocs = new List<string>();
    List<string> runnableTrajectories = new List<string>();
    foreach (var prm in app.primitives)
    {
        if (prm.Value.type == "move")
        {
            var cast = (MovePrimitive)(prm.Value);

            if (cast.startLocationUuid != null && usedLocs.IndexOf(cast.startLocationUuid) < 0)
            {
                usedLocs.Add(cast.startLocationUuid);
            } 

            if (cast.endLocationUuid != null && usedLocs.IndexOf(cast.endLocationUuid) < 0)
            {
                usedLocs.Add(cast.endLocationUuid);
            }

            if (cast.runnableTrajectoryUuid != null)
            {
                runnableTrajectories.Add(cast.runnableTrajectoryUuid);
            }             
        }
    }

    // display all locations (highlight used)
    foreach (var loc in app.locations)
    {
        var use = usedLocs.IndexOf(loc.Key) >= 0;
        if (use)
        {
            joints.Add(loc.Value.joints);
        }

        GameObject obj = Instantiate(LocationPrefab, GetSceneFrame(), false);
        obj.transform.Translate(loc.Value.position);
        obj.transform.Rotate(loc.Value.orientation.eulerAngles);

        LocationScript ls = obj.GetComponent<LocationScript>();
        ls.selectedCallback = LocationClicked;
        ls.Initialize(loc.Key, loc.Value.name);
        ls.Selected(use);

        locations.Add(ls);
    }

    // render the true trajectory - (if not available render line trajectory)
    foreach (var tid in runnableTrajectories)
    {
        var trace = app.TraceTrajectory(tid);
        if (trace != null)
        {
            GameObject tObj = Instantiate(TrajectoryPrefab, GetSceneFrame(), false);
            TrajectoryRenderScript tts = tObj.GetComponent<TrajectoryRenderScript>();
            tts.Initialize(activeUuid, trace, false);
            traceTrajectories.Add(tts);
        }
        else
        {
            var line = app.LineTrajectory(tid);
            if (line != null)
            {
                GameObject lObj = Instantiate(TrajectoryPrefab, GetSceneFrame(), false);
                TrajectoryRenderScript lts = lObj.GetComponent<TrajectoryRenderScript>();
                lts.Initialize(tid, line, false);
                lineTrajectories.Add(lts);
            }
        }
    }

    robotMarkerManager.CreateMarkerVisualization(joints);
}

private void RenderOperateScene()
{
    // TODO (ignoring for now)
}

private void RenderMoveScene()
{
    List<double[]> joints = new List<double[]>();

    // get primitive from active uuid
    var prm = (MovePrimitive)(app.GetPrimitive(activeUuid));

    // select locs corresponding to primitive
    List<string> selectedLocs = new List<string>();
    if (prm.startLocationUuid != null)
    {
        selectedLocs.Add(prm.startLocationUuid);
    }
    if (prm.endLocationUuid != null)
    {
        selectedLocs.Add(prm.endLocationUuid);
    }

    // display all locations
    foreach (var loc in app.locations)
    {
        var use = selectedLocs.IndexOf(loc.Key) >= 0;
        if (use)
        {
            joints.Add(loc.Value.joints);
        }

        GameObject obj = Instantiate(LocationPrefab, GetSceneFrame(), false);
        obj.transform.Translate(loc.Value.position);
        obj.transform.Rotate(loc.Value.orientation.eulerAngles);

        LocationScript ls = obj.GetComponent<LocationScript>();
        ls.selectedCallback = LocationClicked;
        ls.Initialize(loc.Key, loc.Value.name);
        ls.Selected(use);

        locations.Add(ls);
    }

    // display all trace trajectories (if un-available display line trajectory)
    foreach (var tid in prm.trajectoryUuids)
    {
        var trace = app.TraceTrajectory(tid);
        if (trace != null)
        {
            GameObject tObj = Instantiate(TrajectoryPrefab, GetSceneFrame(), false);
            TrajectoryRenderScript tts = tObj.GetComponent<TrajectoryRenderScript>();
            tts.Initialize(tid, trace, true);
            traceTrajectories.Add(tts);
        }
        else
        {
            var line = app.LineTrajectory(tid);
            if (line != null)
            {
                GameObject lObj = Instantiate(TrajectoryPrefab, GetSceneFrame(), false);
                TrajectoryRenderScript lts = lObj.GetComponent<TrajectoryRenderScript>();
                lts.Initialize(tid, line, false);
                lineTrajectories.Add(lts);
            }
        }
    }

    robotMarkerManager.CreateMarkerVisualization(joints);
}

private void RenderTrajectoryScene()
{
    List<double[]> joints = new List<double[]>();

    // get trajectory from active uuid
    var traj = app.GetTrajectory(activeUuid);

    // display locations corresponding to trajectory
    var startLoc = app.GetLocation(traj.startLocationUuid);
    GameObject startLocObj = Instantiate(LocationPrefab, GetSceneFrame(), false);
    startLocObj.transform.Translate(startLoc.position);
    startLocObj.transform.Rotate(startLoc.orientation.eulerAngles);

    joints.Add(startLoc.joints);

    LocationScript startLocLs = startLocObj.GetComponent<LocationScript>();
    startLocLs.selectedCallback = LocationClicked;
    startLocLs.Initialize(startLoc.uuid, startLoc.name);
    startLocLs.Selected(true);

    locations.Add(startLocLs);

    var endLoc = app.GetLocation(traj.endLocationUuid);
    GameObject endLocObj = Instantiate(LocationPrefab, GetSceneFrame(), false);
    endLocObj.transform.Translate(endLoc.position);
    endLocObj.transform.Rotate(endLoc.orientation.eulerAngles);

    joints.Add(endLoc.joints);

    LocationScript endLocLs = endLocObj.GetComponent<LocationScript>();
    endLocLs.selectedCallback = LocationClicked;
    endLocLs.Initialize(endLoc.uuid, endLoc.name);
    endLocLs.Selected(true);

    locations.Add(endLocLs);

    // display waypoints corresponding to trajectory
    foreach (var wpId in traj.waypointUuids)
    {
        var wp = app.GetWaypoint(wpId);
        GameObject obj = Instantiate(WaypointPrefab, GetSceneFrame(), false);
        obj.transform.Translate(wp.position);
        obj.transform.Rotate(wp.orientation.eulerAngles);

        joints.Add(wp.joints);

        WaypointScript ws = obj.GetComponent<WaypointScript>();
        ws.selectedCallback = WaypointClicked;
        ws.Initialize(wp.uuid, wp.name);
        ws.Selected(false);

        waypoints.Add(ws);
    }

    // display trace trajectory (if available)
    var trace = app.TraceTrajectory(activeUuid);
    if (trace != null)
    {
        GameObject tObj = Instantiate(TrajectoryPrefab, GetSceneFrame(), false);
        TrajectoryRenderScript tts = tObj.GetComponent<TrajectoryRenderScript>();
        tts.Initialize(activeUuid, trace, true);
        traceTrajectories.Add(tts);
    }

    // display line trajectory
    var line  = app.LineTrajectory(activeUuid);
    if (line != null)
    {
        GameObject lObj = Instantiate(TrajectoryPrefab, GetSceneFrame(), false);
        TrajectoryRenderScript lts = lObj.GetComponent<TrajectoryRenderScript>();
        lts.Initialize(activeUuid, line, false);
        lineTrajectories.Add(lts);
    }

    robotMarkerManager.CreateMarkerVisualization(joints);
}

private void RenderGripScene()
{
    // TODO (ignoring for now)
}

private void RenderDelayScene()
{
    // TODO (ignoring for now)
}

private void RenderMachineScene()
{
    // TODO (ignoring for now)
}

//=========================================================================
//  Scene Object(s) Callbacks
//=========================================================================

public void LocationClicked(LocationScript ls)
{
    locationCallback?.Invoke(ls.uuid);
}

public void WaypointClicked(WaypointScript ws)
{
    waypointCallback?.Invoke(ws.uuid);
}

//=========================================================================
//  Utility Functions
//=========================================================================

private int FindLocationById(string uuid)
{
    int index = -1;
    for (int i = 0; i < locations.Count; i++)
    {
        if (locations[i].uuid == uuid)
        {
            index = i;
            break;
        }
    }
    return index;
}

private int FindWaypointById(string uuid)
{
    int index = -1;
    for (int i = 0; i < waypoints.Count; i++)
    {
        if (waypoints[i].uuid == uuid)
        {
            index = i;
            break;
        }
    }
    return index;
}
*/
