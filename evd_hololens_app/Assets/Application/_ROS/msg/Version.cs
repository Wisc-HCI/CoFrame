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
public class Version : Message
{
[JsonIgnore]
public const string RosMessageName = "cobots_core/Version";

public Time timestamp;
public String uuid;
public String source;

public Version()
{
timestamp = new Time();
uuid = new String();
source = new String();
}
}
}

