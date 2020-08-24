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
    public class SetDataRequest : Message
    {
        [JsonIgnore]
        public const string RosMessageName = "cobots_core/SetData";

        public String data;
        public Version tag;

        public SetDataRequest(String _data, Version _tag)
        {
            data = _data;
            tag = _tag;
        }
    }

    public class SetDataResponse : Message
    {
        [JsonIgnore]
        public const string RosMessageName = "cobots_core/SetData";

        public bool status;
        public String errors;
        public String message;
    }
}
