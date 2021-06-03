using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.Messages;
using RosSharp.RosBridgeClient.Messages.Standard;

public class OperateControlsScript : MonoBehaviour
{
    public GameObject ExpertViewOptionsMenuPanel;

    public Button StepBackwardButton;
    public Button StopButton;
    public Button PlayButton;
    public Button PauseButton;
    public Button ResetButton;
    public Button StepForwardButton;

    private bool atStart = true;
    private bool atEnd = false;

    private RosSocket rosSocket;

    private string rosUsePhysicalRobotPub;
    private string rosUseSimulatedRobotPub;

    private string rosStepBackwardPub;
    private string rosStopPub;
    private string rosPlayPub;
    private string rosPausePub;
    private string rosResetPub;
    private string rosStepForwardPub;
    private string rosFreedrivePub;

    private string rosProgramAtStartSub;
    private string rosProgramAtEndSub;
    private string rosLockoutSub;

    private void Start()
    {
        StepBackwardButton.interactable = false;
        StopButton.interactable = false;
        PlayButton.interactable = true;
        PauseButton.interactable = false;
        ResetButton.interactable = true;
        StepForwardButton.interactable = true;

        rosSocket = GameObject.FindGameObjectWithTag("ROS Bridge").GetComponent<RosConnector>().RosSocket;

        // publishers
        rosUsePhysicalRobotPub = rosSocket.Advertise<Bool>("robot_control_server/use_physical_robot");
        rosUseSimulatedRobotPub = rosSocket.Advertise<Bool>("robot_control_server/use_simulated_robot");
        rosStepBackwardPub = rosSocket.Advertise<Empty>("robot_control_server/step_backward");
        rosStopPub = rosSocket.Advertise<Empty>("robot_control_server/stop");
        rosPlayPub = rosSocket.Advertise<Empty>("robot_control_server/play");
        rosPausePub = rosSocket.Advertise<Empty>("robot_control_server/pause");
        rosResetPub = rosSocket.Advertise<Empty>("robot_control_server/reset");
        rosStepForwardPub = rosSocket.Advertise<Empty>("robot_control_server/step_forward");
        rosFreedrivePub = rosSocket.Advertise<Bool>("robot_control_server/freedrive");

        // subscribers
        rosProgramAtStartSub = rosSocket.Subscribe<Bool>("robot_control_server/at_start", ProgramAtStartCallback);
        rosProgramAtEndSub = rosSocket.Subscribe<Bool>("robot_control_server/at_end", ProgramAtEndCallback);
        rosLockoutSub = rosSocket.Subscribe<Bool>("robot_control_server/lockout", LockoutCallback);
    }

    private void ProgramAtStartCallback(Bool msg)
    {
        atStart = msg.data;

        //TODO might need to update controls
    }

    private void ProgramAtEndCallback(Bool msg)
    {
        atEnd = msg.data;

        //TODO might need to update controls
    }

    private void LockoutCallback(Bool msg)
    {
        //TODO need to disable all controls
    }

    public void OnMenuButtonClicked()
    {
        ExpertViewOptionsMenuPanel.SetActive(!ExpertViewOptionsMenuPanel.activeSelf);
    }

    public void OnUsePhysicalRobotToggleClicked(bool val)
    {
        var msg = new Bool();
        msg.data = val;
        rosSocket.Publish(rosUsePhysicalRobotPub, msg);
    }

    public void OnUseSimulatedRobotToggleClicked(bool val)
    {
        var msg = new Bool();
        msg.data = val;
        rosSocket.Publish(rosUseSimulatedRobotPub, msg);
    }

    public void OnStepBackwardButtonClicked()
    {
        StepBackwardButton.interactable = true && !atStart;
        StopButton.interactable = true;
        PlayButton.interactable = true;
        PauseButton.interactable = false;
        ResetButton.interactable = false;
        StepForwardButton.interactable = true && !atEnd;

        var msg = new Empty();
        rosSocket.Publish(rosStepBackwardPub, msg);
    }

    public void OnStepForwardButtonClicked()
    {
        StepBackwardButton.interactable = true && !atStart;
        StopButton.interactable = true;
        PlayButton.interactable = true;
        PauseButton.interactable = false;
        ResetButton.interactable = false;
        StepForwardButton.interactable = true && !atEnd;

        var msg = new Empty();
        rosSocket.Publish(rosStepForwardPub, msg);
    }

    public void OnStopButtonClicked()
    {
        StepBackwardButton.interactable = false;
        StopButton.interactable = false;
        PlayButton.interactable = false;
        PauseButton.interactable = false;
        ResetButton.interactable = true;
        StepForwardButton.interactable = false;

        var msg = new Empty();
        rosSocket.Publish(rosStopPub, msg);
    }

    public void OnPlayButtonClicked()
    {
        StepBackwardButton.interactable = false;
        StopButton.interactable = true;
        PlayButton.interactable = false;
        PauseButton.interactable = true;
        ResetButton.interactable = false;
        StepForwardButton.interactable = false;

        var msg = new Empty();
        rosSocket.Publish(rosPlayPub, msg);
    }

    public void OnResetButtonClicked()
    {
        StepBackwardButton.interactable = false;
        StopButton.interactable = false;
        PlayButton.interactable = true;
        PauseButton.interactable = false;
        ResetButton.interactable = true;
        StepForwardButton.interactable = true && !atEnd;

        var msg = new Empty();
        rosSocket.Publish(rosResetPub, msg);
    }

    public void OnPauseButtonClicked()
    {
        StepBackwardButton.interactable = true && !atStart;
        StopButton.interactable = true;
        PlayButton.interactable = true;
        PauseButton.interactable = false;
        ResetButton.interactable = false;
        StepForwardButton.interactable = true && !atEnd;

        var msg = new Empty();
        rosSocket.Publish(rosPausePub, msg);
    }
}
