/*
This message class is generated automatically with 'ServiceMessageGenerator' of ROS#
*/ 

using Newtonsoft.Json;
using RosSharp.RosBridgeClient.Messages.Geometry;
using RosSharp.RosBridgeClient.Messages.Navigation;
using RosSharp.RosBridgeClient.Messages.Sensor;
using RosSharp.RosBridgeClient.Messages.Standard;
using RosSharp.RosBridgeClient.Messages.Actionlib;

namespace RosSharp.RosBridgeClient.Services
{
    public class GetOptionsRequest : Message
    {
        [JsonIgnore]
        public const string RosMessageName = "cobots_core/GetOptions";

        public GetOptionsRequest() {}
    }

    public class GetOptionsResponse : Message
    {
        [JsonIgnore]
        public const string RosMessageName = "cobots_core/GetOptions";

        public String[] options;
    }
}