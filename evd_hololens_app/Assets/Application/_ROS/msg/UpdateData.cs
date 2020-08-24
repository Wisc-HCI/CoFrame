/*
This message class is generated automatically with 'SimpleMessageGenerator' of ROS#
*/ 

using Newtonsoft.Json;
using RosSharp.RosBridgeClient.Messages.Geometry;
using RosSharp.RosBridgeClient.Messages.Navigation;
using RosSharp.RosBridgeClient.Messages.Sensor;
using RosSharp.RosBridgeClient.Messages.Standard;
using RosSharp.RosBridgeClient.Messages.Actionlib;

namespace RosSharp.RosBridgeClient.Messages
{
public class UpdateData : Message
{
[JsonIgnore]
public const string RosMessageName = "cobots_core/UpdateData";

public String data;
public String action;
public String changes;
public Version currentTag;
public Version previousTag;

public UpdateData()
{
data = new String();
action = new String();
changes = new String();
currentTag = new Version();
previousTag = new Version();
}
}
}

