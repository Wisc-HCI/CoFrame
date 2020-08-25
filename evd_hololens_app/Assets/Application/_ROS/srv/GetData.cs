/*
This message class is generated automatically with 'ServiceMessageGenerator' of ROS#
*/ 

using Newtonsoft.Json;
using RosSharp.RosBridgeClient.Messages;
using RosSharp.RosBridgeClient.Messages.Geometry;
using RosSharp.RosBridgeClient.Messages.Navigation;
using RosSharp.RosBridgeClient.Messages.Sensor;
using RosSharp.RosBridgeClient.Messages.Standard;
using RosSharp.RosBridgeClient.Messages.Actionlib;

namespace RosSharp.RosBridgeClient.Services
{
public class GetDataRequest : Message
{
[JsonIgnore]
public const string RosMessageName = "cobots_core/GetData";

public bool all;
public String data;

public GetDataRequest(bool _all, String _data){all = _all;
data = _data;
}
}

public class GetDataResponse : Message
{
[JsonIgnore]
public const string RosMessageName = "cobots_core/GetData";

public String data;
public Version tag;
public bool error;
public String errors;
public String message;
}
}

