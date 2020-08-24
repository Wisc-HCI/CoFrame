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
    public class LoadDataRequest : Message
    {
        [JsonIgnore]
        public const string RosMessageName = "cobots_core/LoadData";

        String filename;

        public LoadDataRequest(String _filename)
        {
            filename = _filename;
        }
    }

    public class LoadDataResponse : Message
    {
        [JsonIgnore]
        public const string RosMessageName = "cobots_core/LoadData";

        public bool status;
        public String message;
    }
}
