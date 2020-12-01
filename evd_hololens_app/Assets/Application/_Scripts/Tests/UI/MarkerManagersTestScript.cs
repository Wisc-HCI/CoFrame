using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class MarkerManagersTestScript : MonoBehaviour
{
    public RobotMarkerManagerScript robotMarkerManager;
    public InteractiveMarkerManagerScript interactiveMarkerManager;

    public Dropdown typeDropdown;
    public Dropdown optionsDropdown;

    public GameObject SelectedPanel;
    public GameObject NonePanel;
    public Text AddNewButtonText;
    public GameObject DisableMainPanel;
    public JointStateEditorScript jointEditor;
    public MarkerPoseEditorScript poseEditor;

    private string type = "start";
    private string activeId = null;
    private EvD.Program program = new EvD.Program();
    private EvD.AttributeTraceProcessor traceProcessor = new EvD.AttributeTraceProcessor();
    private EvD.Container<EvD.Waypoint> waypointContainer = new EvD.Container<EvD.Waypoint>();
    private EvD.Container<EvD.Location> locationContainer = new EvD.Container<EvD.Location>();

    public Transform spawnPoint;

    private void Awake()
    {
        traceProcessor.SubscribeToType("node.primitive.container<EvD.Waypoint>.", ContainerCallback);
        traceProcessor.SubscribeToType("node.primitive.container<EvD.Location>.", ContainerCallback);
        traceProcessor.SubscribeToType("node.pose.waypoint.location.", MarkerCallback);
        traceProcessor.SubscribeToType("node.pose.waypoint.", MarkerCallback);

        program.changesCallback = traceProcessor.ProcessTrace;
        program.AddPrimitive(waypointContainer);
        program.AddPrimitive(locationContainer);

        jointEditor.waypoint = null;
        jointEditor.closeCallback = OnJointStateEditorCallback;
        jointEditor.gameObject.SetActive(false);

        poseEditor.waypoint = null;
        poseEditor.closeCallback = OnMarkerEditorCallback;
        poseEditor.gameObject.SetActive(false);

        DisableMainPanel.SetActive(false);

        OnTypeDropdownChange();
        OnOptionsDropdownChange();
    }

    private void Update()
    {
        bool isLocType = type == "location";

        foreach (var w in waypointContainer.values)
        {
            if (robotMarkerManager.GetVisible(w.uuid) != !isLocType)
            {
                robotMarkerManager.SetVisible(w.uuid, !isLocType);
            }
        }

        foreach (var l in locationContainer.values)
        {
            if (robotMarkerManager.GetVisible(l.uuid) != isLocType)
            {
                robotMarkerManager.SetVisible(l.uuid, isLocType);
            }
        }
    }

    public void OnTypeDropdownChange()
    {
        if (typeDropdown.options[typeDropdown.value].text.ToLower() == "location")
        {
            if (type != "location")
            {
                type = "location";

                AddNewButtonText.text = "Add New Location";

                List<Dropdown.OptionData> options = new List<Dropdown.OptionData>();
                options.Add(new Dropdown.OptionData("-- None --"));
                foreach (var loc in locationContainer.values)
                {
                    options.Add(new Dropdown.OptionData(loc.uuid));
                }

                optionsDropdown.options = options;
                optionsDropdown.value = 0;
            }
        }
        else
        {
            if (type != "waypoint")
            {
                type = "waypoint";

                AddNewButtonText.text = "Add New Waypoint";

                List<Dropdown.OptionData> options = new List<Dropdown.OptionData>();
                options.Add(new Dropdown.OptionData("-- None --"));
                foreach (var way in waypointContainer.values)
                {
                    options.Add(new Dropdown.OptionData(way.uuid));
                }

                optionsDropdown.options = options;
                optionsDropdown.value = 0;
            }   
        }
    }

    public void OnOptionsDropdownChange()
    {
        if (optionsDropdown.options[optionsDropdown.value].text.ToLower() == "-- none --")
        {
            SelectedPanel.SetActive(false);
            NonePanel.SetActive(true);
            activeId = null;
        }
        else
        {
            SelectedPanel.SetActive(true);
            NonePanel.SetActive(false);
            if (type == "location")
            {
                activeId = locationContainer.values[optionsDropdown.value - 1].uuid;
            }
            else
            {
                activeId = waypointContainer.values[optionsDropdown.value - 1].uuid;
            }
        }
    }

    public void OnAddNewButtonPress()
    {
        if (typeDropdown.options[typeDropdown.value].text.ToLower() == "location")
        {
            // New location
            EvD.Location loc = new EvD.Location(EvD.Position.FromUnity(spawnPoint.localPosition), EvD.Orientation.FromUnity(spawnPoint.localRotation));
            loc.name = loc.uuid;
            locationContainer.Add(loc);

            List<Dropdown.OptionData> newOptions = new List<Dropdown.OptionData>();
            newOptions.Add(new Dropdown.OptionData(loc.uuid));
            optionsDropdown.AddOptions(newOptions);
        }
        else
        {
            // New waypoint
            EvD.Waypoint way = new EvD.Waypoint(EvD.Position.FromUnity(spawnPoint.localPosition), EvD.Orientation.FromUnity(spawnPoint.localRotation));
            way.name = way.uuid;
            waypointContainer.Add(way);

            List<Dropdown.OptionData> newOptions = new List<Dropdown.OptionData>();
            newOptions.Add(new Dropdown.OptionData(way.uuid));
            optionsDropdown.AddOptions(newOptions);
        }

        optionsDropdown.value = optionsDropdown.options.Count;
    }

    public void OnEditMarkerPoseButton()
    {
        if (type == "location")
        {
            poseEditor.waypoint = locationContainer.Get(activeId);
        }
        else
        {
            poseEditor.waypoint = waypointContainer.Get(activeId);
        }

        DisableMainPanel.SetActive(true);
        poseEditor.gameObject.SetActive(true);
    }

    public void OnEditJointsButton()
    {
        if (type == "location")
        {
            jointEditor.waypoint = locationContainer.Get(activeId);
        }
        else
        {
            jointEditor.waypoint = waypointContainer.Get(activeId);
        }

        DisableMainPanel.SetActive(true);
        jointEditor.gameObject.SetActive(true);
    }

    public void OnDeleteButton()
    {
        if (type == "location")
        {
            locationContainer.Delete(activeId);

            List<Dropdown.OptionData> options = new List<Dropdown.OptionData>();
            options.Add(new Dropdown.OptionData("-- None --"));
            foreach (var loc in locationContainer.values)
            {
                options.Add(new Dropdown.OptionData(loc.uuid));
            }

            optionsDropdown.options = options;
            optionsDropdown.value -= 1;
        }
        else
        {
            waypointContainer.Delete(activeId);

            List<Dropdown.OptionData> options = new List<Dropdown.OptionData>();
            options.Add(new Dropdown.OptionData("-- None --"));
            foreach (var way in waypointContainer.values)
            {
                options.Add(new Dropdown.OptionData(way.uuid));
            }

            optionsDropdown.options = options;
            optionsDropdown.value -= 1;
        }
        
        OnOptionsDropdownChange();
    }

    public void OnJointStateEditorCallback()
    {
        jointEditor.waypoint = null;
        jointEditor.gameObject.SetActive(false);

        DisableMainPanel.SetActive(false);
    }

    public void OnMarkerEditorCallback()
    {
        poseEditor.waypoint = null;
        poseEditor.gameObject.SetActive(false);

        DisableMainPanel.SetActive(false);
    }

    public void ContainerCallback(Dictionary<string,string> entry)
    {
        if (entry["verb"] == "add")
        {
            var m = (EvD.Waypoint)(program.cache.Get(entry["child_uuid"]));
            robotMarkerManager.OnWaypointAdd(m);
            interactiveMarkerManager.OnWaypointAdd(m);
        }
        else if (entry["verb"] == "delete")
        {
            var id = entry["child_uuid"];
            robotMarkerManager.OnWaypointDelete(id);
            interactiveMarkerManager.OnWaypointDelete(id);
        }
    }

    public void MarkerCallback(Dictionary<string,string> entry)
    {
        if (entry["verb"] == "set")
        {
            if (entry["attribute"] == "joints")
            {
                robotMarkerManager.OnWaypointUpdate((EvD.Waypoint)(program.cache.Get(entry["uuid"])));
            }
            
            if (entry["attribute"] == "position" || entry["attribute"] == "orientation" || entry["attribute"] == "name")
            {
                interactiveMarkerManager.OnWaypointUpdate((EvD.Waypoint)(program.cache.Get(entry["uuid"])));
            }
        }
    }
}
