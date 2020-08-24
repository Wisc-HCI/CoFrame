using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.Messages.Standard;

public class ExpertViewMenuScript : MonoBehaviour
{
    public Toggle endEffectorToggle;
    public Toggle jointToggle;
    public Toggle toolToggle;
    public Toggle robotMarkersToggle;
    public Toggle reachSphereToggle;
    public Toggle payloadToggle;

    private RosSocket rosSocket;

    private string rosEndEffectorPathOptionPub;
    private string rosJointPathOptionPub;
    private string rosToolPathOptionPub;
    private string rosRobotMarkersOptionPub;
    private string rosReachSphereOptionPub;
    private string rosPayloadOptionPub;

    private void Start()
    {
        rosSocket = GameObject.FindGameObjectWithTag("ROS Bridge").GetComponent<RosConnector>().RosSocket;

        // publishers
        rosEndEffectorPathOptionPub = rosSocket.Advertise<Bool>("environment/view_options/end_effector_path");
        rosJointPathOptionPub = rosSocket.Advertise<Bool>("environment/view_options/joint_path");
        rosToolPathOptionPub = rosSocket.Advertise<Bool>("environment/view_options/tool_path");
        rosRobotMarkersOptionPub = rosSocket.Advertise<Bool>("environment/view_options/robot_markers");
        rosReachSphereOptionPub = rosSocket.Advertise<Bool>("environment/view_options/reach_sphere");
        rosPayloadOptionPub = rosSocket.Advertise<Bool>("environment/view_options/payload");

        // send initial state
        var msg = new Bool();
        msg.data = endEffectorToggle.isOn;
        rosSocket.Publish(rosEndEffectorPathOptionPub, msg);
        msg.data = jointToggle.isOn;
        rosSocket.Publish(rosJointPathOptionPub, msg);
        msg.data = toolToggle.isOn;
        rosSocket.Publish(rosToolPathOptionPub, msg);
        msg.data = robotMarkersToggle.isOn;
        rosSocket.Publish(rosRobotMarkersOptionPub, msg);
        msg.data = reachSphereToggle.isOn;
        rosSocket.Publish(rosReachSphereOptionPub, msg);
        msg.data = payloadToggle.isOn;
        rosSocket.Publish(rosPayloadOptionPub, msg);
    }

    public void OnEndEffectorPathToggleClicked(bool val)
    {
        var msg = new Bool();
        msg.data = val;
        rosSocket.Publish(rosEndEffectorPathOptionPub, msg);
    }

    public void OnJointPathToggleClicked(bool val)
    {
        var msg = new Bool();
        msg.data = val;
        rosSocket.Publish(rosJointPathOptionPub, msg);
    }

    public void OnToolPathToggleClicked(bool val)
    {
        var msg = new Bool();
        msg.data = val;
        rosSocket.Publish(rosToolPathOptionPub, msg);
    }

    public void OnRobotMarkersToggleClicked(bool val)
    {
        var msg = new Bool();
        msg.data = val;
        rosSocket.Publish(rosRobotMarkersOptionPub, msg);
    }

    public void OnReachSphereToggleClicked(bool val)
    {
        var msg = new Bool();
        msg.data = val;
        rosSocket.Publish(rosReachSphereOptionPub, msg);
    }

    public void OnPayloadToggleClicked(bool val)
    {
        var msg = new Bool();
        msg.data = val;
        rosSocket.Publish(rosPayloadOptionPub, msg);
    }
}
