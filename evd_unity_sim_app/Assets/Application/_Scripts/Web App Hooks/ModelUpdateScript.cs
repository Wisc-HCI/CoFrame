using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using EvD;
using EvD.Data;
using EvD.Environment;

public class ModelUpdateScript : MonoBehaviour
{
    public OccupancyZoneManagerScript OccupancyZoneManager;
    public ReachSphereScript ReachSphereManager;
    public CollisionMeshManagerScript CollisionMeshManager;
    public PinchPointManagerScript PinchPointManager;
    public TrajectoryManagerScript TrajectoriesManager;

    private EnvironmentContext environment;
    private AttributeTraceProcessor changesProcessor;
    
    private void Start() 
    {
        SetupAttributeProcessor();    
        DefineTestEnvironment();
    }

    public void UpdateModel(string data)
    {
        // TODO parse data (JSON string) into model (environment)
        Debug.Log(data);
    }

    public void SetupAttributeProcessor()
    {
        changesProcessor = new AttributeTraceProcessor();
        changesProcessor.SubscribeToType(ReachSphere.FullTypeString(),ReachSphereUpdated);
        changesProcessor.SubscribeToType(CollisionMesh.FullTypeString(),CollisionMeshesUpdated);
        changesProcessor.SubscribeToType(PinchPoint.FullTypeString(),PinchPointsUpdated);
        changesProcessor.SubscribeToType(OccupancyZone.FullTypeString(),OccupancyZonesUpdated);
    }

    public void DefineTestEnvironment()
    {
        /*
        // Simple test program for checking the managers
        var reachSphere = new ReachSphere(0.75f, new Position(0,0,0));
        
        List<OccupancyZone> zones = new List<OccupancyZone>();
        var robotZone = new OccupancyZone(OccupancyZone.ROBOT_TYPE, 0, 0, 1, 1, -0.75f);
        zones.Add(robotZone);

        for (int i=0; i<2; i++) 
        {
            var humanZone = new OccupancyZone(OccupancyZone.HUMAN_TYPE, 0, 1, 1, 1, -0.75f);
            zones.Add(humanZone);
        }

        List<CollisionMesh> meshes = new List<CollisionMesh>();


        List<PinchPoint> points = new List<PinchPoint>();

        
        List<Location> locations = new List<Location>();


        List<Machine> machines =  new List<Machine>();


        List<Thing> things = new List<Thing>();


        List<Trajectory> trajectories = new List<Trajectory>();


        List<Waypoint> waypoints = new List<Waypoint>();

        environment = new EnvironmentContext(reachSphere, points, meshes, zones, locations, 
                                             machines, things, trajectories, waypoints, 
                                             changesProcessor.ProcessTrace);
        
        environment.LateConstructUpdate();
        environment.DeepUpdate();
        */
    }

    public void ReachSphereUpdated(Dictionary<string, string> trace)
    {
        // TODO
        Debug.Log(trace);
    }

    public void CollisionMeshesUpdated(Dictionary<string, string> trace)
    {
        // TODO
        Debug.Log(trace);
    }

    public void PinchPointsUpdated(Dictionary<string, string> trace)
    {
        // TODO
        Debug.Log(trace);
    }

    public void OccupancyZonesUpdated(Dictionary<string, string> trace)
    {
        // TODO
        Debug.Log(trace);
    }
}
