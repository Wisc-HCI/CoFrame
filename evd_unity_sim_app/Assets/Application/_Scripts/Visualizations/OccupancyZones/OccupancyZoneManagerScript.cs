using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class OccupancyZoneManagerScript : MonoBehaviour
{
    public HumanOccupancyZoneScript HumanOccupancyZone;
    public OccupancyZoneScript RobotOccupancyZone;

    public System.Action<string, Vector2, Vector2> zoneManipulatedCallback = null; // uuid, position, scale

    [SerializeField]
    private bool humanActive = false;
    [SerializeField]
    private bool robotActive = false;

    public void OnUpdate(EvD.Environment.OccupancyZone zone)
    {
        if (zone.occupancyType == EvD.Environment.OccupancyZone.HUMAN_TYPE)
        {
            humanActive = true;
            HumanOccupancyZone.model = zone;
            HumanOccupancyZone.manipulatedCallback = zoneManipulatedCallback;
        }
        else // must be robot type
        {
            robotActive = true;
            RobotOccupancyZone.model = zone;
        }
    }

    public void OnDelete(string uuid)
    {
        if (HumanOccupancyZone.uuid == uuid)
        {
            HumanOccupancyZone.visible = false;
            HumanOccupancyZone.controlsVisible = false;
            HumanOccupancyZone.manipulatedCallback = null;
            HumanOccupancyZone.model = null;
            humanActive = false;
        }

        if (RobotOccupancyZone.uuid == uuid)
        {
            RobotOccupancyZone.gameObject.SetActive(false);
            RobotOccupancyZone.model = null;
            robotActive = false;
        }
    }

    public void SetVisible(string uuid, bool val)
    {
        if (HumanOccupancyZone.uuid == uuid)
        {
            HumanOccupancyZone.gameObject.SetActive(humanActive && val);
            HumanOccupancyZone.visible = humanActive && val;
        }
        else if (RobotOccupancyZone.uuid == uuid)
        {
            RobotOccupancyZone.gameObject.SetActive(robotActive && val);
        }
    }

    public void SetVisible(bool val)
    {
        HumanOccupancyZone.gameObject.SetActive(humanActive && val);
        HumanOccupancyZone.visible = humanActive && val;
        RobotOccupancyZone.gameObject.SetActive(robotActive && val);
    }

    public bool GetVisible(string uuid)
    {
        if (uuid == HumanOccupancyZone.uuid)
        {
            return HumanOccupancyZone.visible;
        }
        else if (uuid == RobotOccupancyZone.uuid)
        {
            return RobotOccupancyZone.gameObject.activeSelf;
        }
        else
        {
            return false;
        }
    }

    public void SetControlsVisible(bool val)
    {
        HumanOccupancyZone.controlsVisible = val;
    }

    public bool GetControlsVisible()
    {
        return HumanOccupancyZone.controlsVisible;
    }

    public OccupancyZoneScript GetZone(string uuid)
    {
        if (uuid == HumanOccupancyZone.uuid)
        {
            return HumanOccupancyZone;
        }
        else if (uuid == RobotOccupancyZone.uuid)
        {
            return RobotOccupancyZone;
        }
        else
        {
            throw new System.Exception("Uuid not found within set of zones");
        }
    }

    public void SetHeight(float height)
    {
        HumanOccupancyZone.SetGroundHeight(height);
        RobotOccupancyZone.SetGroundHeight(height);
    }
}
