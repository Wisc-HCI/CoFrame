using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class OccupancyZoneManagerTestScript : MonoBehaviour
{
    public OccupancyZoneManagerScript zoneManager;

    public FloatSliderScript positionX;
    public FloatSliderScript positionZ;
    public FloatSliderScript scaleX;
    public FloatSliderScript scaleZ;

    public Toggle humanVisible;
    public Toggle controlsVisible;
    public Toggle robotVisible;

    private EvD.OccupancyZone human = new EvD.OccupancyZone("human");
    private EvD.OccupancyZone robot = new EvD.OccupancyZone("robot");

    [SerializeField]
    private bool visible = false;
    [SerializeField]
    private bool cVisible = false;

    private void Awake()
    {
        positionX.valueChangedCallback = OnPositionValueSliderChange;
        positionZ.valueChangedCallback = OnPositionValueSliderChange;
        scaleX.valueChangedCallback = OnScaleValueSliderChange;
        scaleZ.valueChangedCallback = OnScaleValueSliderChange;

        zoneManager.zoneManipulatedCallback = OnZoneChange;
    }

    private void Start()
    {
        zoneManager.OnUpdate(human);
        zoneManager.OnUpdate(robot);

        zoneManager.SetVisible(visible);
        zoneManager.SetControlsVisible(cVisible);
        humanVisible.isOn = zoneManager.GetVisible(human.uuid);
        robotVisible.isOn = zoneManager.GetVisible(robot.uuid);
        controlsVisible.isOn = zoneManager.GetControlsVisible();
    }

    public void OnSetVisible()
    {
        visible = !visible;
        zoneManager.SetVisible(visible);
        humanVisible.isOn = zoneManager.GetVisible(human.uuid);
        robotVisible.isOn = zoneManager.GetVisible(robot.uuid);
    }

    public void OnSetControlsVisible()
    {
        cVisible = !cVisible;
        zoneManager.SetControlsVisible(cVisible);
        controlsVisible.isOn = zoneManager.GetControlsVisible();
    }

    public void OnHumanToggle()
    {
        zoneManager.SetVisible(human.uuid, humanVisible.isOn);
        humanVisible.isOn = zoneManager.GetVisible(human.uuid);
    }

    public void OnRobotToggle()
    {
        zoneManager.SetVisible(robot.uuid, robotVisible.isOn);
        robotVisible.isOn = zoneManager.GetVisible(robot.uuid);
    }

    public void OnPositionValueSliderChange(string name, float value)
    {
        zoneManager.GetZone(human.uuid).SetPosition(new Vector2(positionX.value, positionZ.value));
    }

    public void OnScaleValueSliderChange(string name, float value)
    {
        zoneManager.GetZone(human.uuid).SetScale(new Vector2(scaleX.value, scaleZ.value));
    }

    public void OnZoneChange(string uuid, Vector2 position, Vector2 scale)
    {
        positionX.value = position.x;
        positionZ.value = position.y;
        scaleX.value = scale.x;
        scaleZ.value = scale.y;
    }
}
