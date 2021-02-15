using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class OccupancyZoneManagerScript : MonoBehaviour
{
    public GameObject HumanOccupancyZonePrefab = null;

    public Dictionary<string,HumanOccupancyZoneScript> HumanOccupancyZones = new Dictionary<string,HumanOccupancyZoneScript>();
    public OccupancyZoneScript RobotOccupancyZone = null;

    [SerializeField]
    private bool humanActive = false;
    [SerializeField]
    private bool robotActive = false;

    private void Start() {
        foreach (var entry in HumanOccupancyZones) {
            entry.Value.gameObject.SetActive(humanActive);
            entry.Value.visible = humanActive;
        }
        RobotOccupancyZone.gameObject.SetActive(robotActive);
    }

    public void OnCreate(EvD.Environment.OccupancyZone zone) 
    {
        if (zone.occupancyType == EvD.Environment.OccupancyZone.HUMAN_TYPE)
        {
            var obj = Instantiate(HumanOccupancyZonePrefab, transform, false);
            var hz = obj.GetComponent<HumanOccupancyZoneScript>();
            hz.model = zone;

            HumanOccupancyZones.Add(hz.uuid, hz);
            humanActive = true;
        }
        else // must be robot type
        {
            OnUpdate(zone); // We never create robot zones
        }
    }

    public void OnUpdate(EvD.Environment.OccupancyZone zone)
    {
        if (zone.occupancyType == EvD.Environment.OccupancyZone.HUMAN_TYPE)
        {
            if (HumanOccupancyZones.ContainsKey(zone.uuid)) 
            {
                humanActive = true;
                HumanOccupancyZones[zone.uuid].model = zone;
            }
            else
            {
                OnCreate(zone);
            }
        }
        else // must be robot type
        {
            robotActive = true;
            RobotOccupancyZone.model = zone;
        }
    }

    public void OnDelete(string uuid)
    {
        if (RobotOccupancyZone.uuid == uuid)
        {
            RobotOccupancyZone.gameObject.SetActive(false);
            RobotOccupancyZone.model = null;
            robotActive = false;
        }
        else if (HumanOccupancyZones.ContainsKey(uuid))
        {
            Destroy(HumanOccupancyZones[uuid].gameObject);
            HumanOccupancyZones.Remove(uuid);

            if (HumanOccupancyZones.Count <= 0) 
            {
                humanActive = false;
            }
        }
    }

    public void SetVisible(string uuid, bool val)
    {
        if (RobotOccupancyZone.uuid == uuid)
        {
            bool state = robotActive && val;
            RobotOccupancyZone.gameObject.SetActive(state);
        }
        else if (HumanOccupancyZones.ContainsKey(uuid))
        {
            bool state = humanActive && val;
            HumanOccupancyZones[uuid].gameObject.SetActive(state);
            HumanOccupancyZones[uuid].visible = state;
        }
    }

    public void SetVisible(bool val)
    {
        bool humanState = humanActive && val;
        bool robotState = robotActive && val;

        foreach (var entry in HumanOccupancyZones) 
        {
            entry.Value.gameObject.SetActive(humanState);
            entry.Value.visible = humanState;
        }
        
        RobotOccupancyZone.gameObject.SetActive(robotState);
    }

    public bool GetVisible(string uuid)
    {
        bool retVal = false;

        if (uuid == RobotOccupancyZone.uuid)
        {
            retVal = RobotOccupancyZone.gameObject.activeSelf;
        }
        else if (HumanOccupancyZones.ContainsKey(uuid))
        {
            retVal = HumanOccupancyZones[uuid].gameObject.activeSelf;
        }

        return retVal;
    }

    public OccupancyZoneScript GetZone(string uuid)
    {
        OccupancyZoneScript retVal = null;

        if (uuid == RobotOccupancyZone.uuid) 
        {
            retVal = RobotOccupancyZone;
        } 
        else if (HumanOccupancyZones.ContainsKey(uuid))
        {
            retVal = HumanOccupancyZones[uuid];
        }

        if (retVal == null) 
        {
            throw new System.Exception("Uuid not found within set of zones");
        }
        else
        {
            return retVal;
        } 
    }
}
