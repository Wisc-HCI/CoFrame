using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using EvD;

public class ProgramMoveScript : MonoBehaviour
{
    /*
    private enum SelectionOption
    {
        START_LOC,
        END_LOC,
        NONE
    }

    private SelectionOption selectOption = SelectionOption.NONE;

    public GameObject TrajectoryTilePrefab;

    public GameObject trajectoryTrack;
    public GameObject LocationText;
    public GameObject AddGrayout;
    public GameObject DeleteGrayout;
    public Text StartLocationText;
    public Text EndLocationText;

    public ViewScript view;

    public int maxTrajectories = 9;
    public float stepSize = 10.0f;
    public float displaceScalar = 0.001f;

    public OkayPopupMessageScript warningPanel;

    public string activeTrajectory;
    private List<TrajectoryScript> trajectories = new List<TrajectoryScript>();

    private string currentStartLocId = null;
    private string currentEndLocId = null;
    private string currentUsedTrajectory = null;

    private void Awake()
    {
        view = GetComponent<ViewScript>();
        view.visibleCallbacks.Add(Visible);

        selectOption = SelectionOption.NONE;
    }

    public void Visible(bool val)
    {
        foreach (var t in trajectories)
        {
            Destroy(t.gameObject);
        }
        trajectories.Clear();

        if (view.activeUuid != null)
        {
            var p = (MovePrimitive)(view.app.GetPrimitive(view.activeUuid));

            currentStartLocId = p.startLocationUuid;
            currentEndLocId = p.endLocationUuid;

            LocationsSatisfied();

            // generate trajectories
            foreach (string tid in p.trajectoryUuids)
            {
                Trajectory traj = view.app.GetTrajectory(tid);
                InstantiateTrajectory(traj.name, traj.uuid, p.runnableTrajectoryUuid == traj.uuid);
            }

            OnSelect(activeTrajectory);
        }
    }

    public void AddTrajectoryCallback()
    {
        // add new trajectory to end of list
        if (trajectories.Count + 1 > maxTrajectories)
        {
            warningPanel.DisplayMessage("Reached maximum number of trajectories.", title: "Warning");
        }
        else
        {
            // create new trajectory
            Trajectory traj = new Trajectory("joint");
            view.app.AddTrajectory(view.activeUuid, traj.uuid, traj);

            bool use = trajectories.Count == 0;
            if (use)
            {
                view.app.SetRunnableTrajectoryMovePrimitive(view.activeUuid, traj.uuid);
            }

            InstantiateTrajectory(traj.name, traj.uuid, use);

            // set as active
            OnSelect(traj.uuid);
        }
    }

    public void TrajectorySelectedCallback(string uuid)
    {
        if (uuid != activeTrajectory)
        {
            OnSelect(uuid);
        }
        else
        {
            view.navigation.ParameterizeButtonClicked();
        }
    }

    private void InstantiateTrajectory(string name, string uuid, bool use)
    {
        var t = Instantiate(TrajectoryTilePrefab, trajectoryTrack.transform, false);

        var tf = t.GetComponent<RectTransform>();
        float displace = trajectories.Count * (tf.rect.width + stepSize) * displaceScalar;
        tf.Translate(new Vector3(displace, 0, 0));

        var tos = t.GetComponent<TrajectoryScript>();
        tos.OnInstantiate(name, uuid);
        tos.deleteClickCallback = DeleteTrajectoryCallback;
        tos.selectCallback = TrajectorySelectedCallback;
        tos.shiftLeftClickCallback = OnShiftLeftClick;
        tos.shiftRightClickCallback = OnShiftRightClick;
        tos.useClickCallback = OnUseClick;

        tos.UseTrajectory(use);
        if (use)
        {
            currentUsedTrajectory = uuid;
        }

        trajectories.Add(tos);
    }

    public void DeleteTrajectoryCallback(string uuid = "-1")
    {
        if (uuid == "-1")
        {
            uuid = activeTrajectory;
        }

        // remove active trajectory
        int index = FindTrajectoryIndex(activeTrajectory);
        if (index > -1)
        {
            view.app.DeleteTrajectory(view.activeUuid, uuid);

            // shift back
            Destroy(trajectories[index].gameObject);
            trajectories.RemoveAt(index);
            Shift(index, trajectories.Count, -1);

            // find next active
            if (index - 1 < trajectories.Count && index - 1 >= 0)
            {
                OnSelect(trajectories[index - 1].uuid);
            }
            else if (index - 1 < 0 && trajectories.Count >= 1)
            {
                OnSelect(trajectories[0].uuid);
            }
            else
            {
                OnSelect(null);
            }
        }
    }

    public void OnSelect(string uuid)
    {
        activeTrajectory = uuid;

        bool found = false;
        for (int i=0; i<trajectories.Count; i++)
        {
            bool match = trajectories[i].uuid == uuid;
            found = found || match;
            trajectories[i].ActiveTrajectory(match);
        }

        if (found)
        {
            view.UpdateNavigation("trajectory", uuid);
        }
        else
        {
            view.DisableNavigation();
        }
    }

    private int FindTrajectoryIndex(string uuid)
    {
        int index = -1;
        for (int i = 0; i < trajectories.Count; i++)
        {
            if (trajectories[i].uuid == uuid)
            {
                index = i;
                break;
            }
        }
        return index;
    }

    private List<string> GetAllUuids()
    {
        List<string> uuids = new List<string>();
        foreach (var t in trajectories)
        {
            uuids.Add(t.uuid);
        }
        return uuids;
    }

    private void Shift(int start, int end, int direction)
    {
        for (int j = start; j < end; j++)
        {
            var tf = trajectories[j].GetComponent<RectTransform>();
            float displace = direction * (tf.rect.width + stepSize) * displaceScalar;
            tf.Translate(new Vector3(displace, 0, 0));
        }
    }

    public void OnShiftLeftClick(string uuid)
    {
        int index = FindTrajectoryIndex(uuid);

        if (index - 1 < 0)
        {
            warningPanel.DisplayMessage("At start of list.", title: "Warning");
        }
        else
        {
            Shift(index, index + 1, -1);
            Shift(index - 1, index, 1);

            var obj = trajectories[index];
            trajectories[index] = trajectories[index - 1];
            trajectories[index - 1] = obj;

            view.app.UpdateTrajectoryOrder(view.activeUuid,GetAllUuids());
        }
    }

    public void OnShiftRightClick(string uuid)
    {
        int index = FindTrajectoryIndex(uuid);

        if (index + 1 >= trajectories.Count)
        {
            warningPanel.DisplayMessage("At end of list.", title: "Warning");
        }
        else
        {
            Shift(index, index + 1, 1);
            Shift(index + 1, index + 2, -1);

            var obj = trajectories[index];
            trajectories[index] = trajectories[index + 1];
            trajectories[index + 1] = obj;

            view.app.UpdateTrajectoryOrder(view.activeUuid, GetAllUuids());
        }
    }

    public void StartLocationSelectClicked()
    {
        if (selectOption != SelectionOption.START_LOC)
        {
            StartLocationText.text = "Cancel";
            if (selectOption == SelectionOption.END_LOC) // fix cancel on other button
            {
                EndLocationText.text = "End";
            }
            else // make sure highlighted
            {
                view.app.scene.StartSelectingLocations(LocationSelectedCallback);
            }

            selectOption = SelectionOption.START_LOC;
        }
        else // cancel
        {
            StartLocationText.text = "Start";
            selectOption = SelectionOption.NONE;
            view.app.scene.StopSelectingLocations();
        }
    }

    public void EndLocationSelectClicked()
    {
        if (selectOption != SelectionOption.END_LOC)
        {
            EndLocationText.text = "Cancel";
            if (selectOption == SelectionOption.START_LOC) // fix cancel on other button
            {
                StartLocationText.text = "Start";
            }
            else // make sure highlighted
            {
                view.app.scene.StartSelectingLocations(LocationSelectedCallback);
            }

            selectOption = SelectionOption.END_LOC;
        }
        else // cancel
        {
            EndLocationText.text = "End";
            selectOption = SelectionOption.NONE;
            view.app.scene.StopSelectingLocations();
        }
    }

    public void LocationSelectedCallback(string locId)
    {
        if (selectOption == SelectionOption.START_LOC)
        {
            StartLocationText.text = "Start";
            selectOption = SelectionOption.NONE;
            view.app.SetStartLocationMovePrimitive(view.activeUuid, locId);
            view.app.scene.StopSelectingLocations();

            if (currentStartLocId != null)
            {
                view.app.scene.IndicateSelectedLocation(currentStartLocId, false);
            }
            currentStartLocId = locId;
            view.app.scene.IndicateSelectedLocation(currentStartLocId, true);
        }
        else if (selectOption == SelectionOption.END_LOC)
        {
            EndLocationText.text = "End";
            selectOption = SelectionOption.NONE;
            view.app.SetEndLocationMovePrimitive(view.activeUuid, locId);
            view.app.scene.StopSelectingLocations();

            if (currentEndLocId != null)
            {
                view.app.scene.IndicateSelectedLocation(currentEndLocId, false);
            }
            currentEndLocId = locId;
            view.app.scene.IndicateSelectedLocation(currentEndLocId, true);
        }

        LocationsSatisfied();
    }

    public void LocationsSatisfied()
    {
        bool satisifed = currentEndLocId != null && currentStartLocId != null;
        DeleteGrayout.SetActive(!satisifed);
        AddGrayout.SetActive(!satisifed);
        LocationText.SetActive(!satisifed);
    }

    public void OnUseClick(string trajId)
    {
        view.app.SetRunnableTrajectoryMovePrimitive(view.activeUuid, trajId);

        for (int i=0; i<trajectories.Count; i++)
        {
            if (trajectories[i].uuid == currentUsedTrajectory && currentUsedTrajectory != trajId)
            {
                trajectories[i].UseTrajectory(false);
            }
            else if (trajectories[i].uuid == trajId)
            {
                trajectories[i].UseTrajectory(true);
            }
        }

        currentUsedTrajectory = trajId;
    }
    */
}
