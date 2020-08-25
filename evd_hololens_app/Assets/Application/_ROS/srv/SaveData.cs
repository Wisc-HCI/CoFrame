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
    public class SaveDataRequest : Message
    {
        [JsonIgnore]
        public const string RosMessageName = "cobots_core/SaveData";

        bool use_current_name;
        String filename;

        public SaveDataRequest(bool _use_current_name, String _filename)
        {
            use_current_name = _use_current_name;
            filename = _filename;
        }
    }

    public class SaveDataResponse : Message
    {
        [JsonIgnore]
        public const string RosMessageName = "cobots_core/SaveData";

        public bool status;
        public String message;
    }
}
