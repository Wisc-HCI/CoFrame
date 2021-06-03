using System.Collections.Generic;
using UnityEngine;
using EvD;
using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.Services;
using RosSharp.RosBridgeClient.Messages;
using RosSharp.RosBridgeClient.Messages.Standard;

//TODO need to learn more about Newtonsoft JSON

public class ModelClientScript : MonoBehaviour
{

    private static readonly string SOURCE = "ar";

    private RosSocket rosSocket = null;

    private string rosUpdateProgramSub;
    private string rosUpdateEnvironmentSub;

    private void Awake()
    {

        //rosUpdateProgramSub = rosSocket.Subscribe<UpdateData>("data_server/update_program", UpdateProgramDataCallback);
        //rosUpdateEnvironmentSub = rosSocket.Subscribe<UpdateData>("data_server/update_environment", UpdateEnvironmentDataCallback);

        /*
        rootContext = new Context();
        rootTask = new Task(rootContext);
        
        // For now initialize the locations
        double[] j_st = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
        var l_st = new Location(rootContext, new Vector3(0.125f, 0.15f, 0.8257f), Quaternion.Euler(180, -90, 0), "Start", j_st);
        rootContext.locations.Add(l_st.uuid, l_st);
        double[] j_en = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
        var l_en = new Location(rootContext, new Vector3(0, 0.127f, -0.168f), Quaternion.Euler(180, 0, 0), "End", j_en);
        rootContext.locations.Add(l_en.uuid, l_en);
        double[] j_home = { 0, -1.35, -1.0, -2.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
        var l_home = new Location(rootContext, new Vector3(0.0f, 0.2f, 0.1f), Quaternion.Euler(180, 0, 0), "Home", j_home);
        rootContext.locations.Add(l_home.uuid, l_home);

        
        // For now initialize a program
        rootTask.AddPrimitive(new InitializeTask(rootTask.context, l_home.uuid));
        
        var loop = new Loop(rootTask.context);
        loop.AddPrimitive(new PickAndPlaceTask(loop.context, l_st.uuid, l_home.uuid, l_en.uuid));
        loop.AddPrimitive(new RetractTask(loop.context, l_en.uuid, l_home.uuid));
        loop.AddPrimitive(new CNCProcessTask(loop.context));
        loop.AddPrimitive(new PickAndPlaceTask(loop.context, l_home.uuid, l_en.uuid, l_st.uuid));

        rootTask.AddPrimitive(loop);
        */
    }

    /**
     * Application Interface
     */

    public void LoadApplication(string filename)
    {
        var strWrp = new String();
        strWrp.data = filename;
        var request = new LoadDataRequest(strWrp);
        rosSocket.CallService<LoadDataRequest, LoadDataResponse>("data_server/load_application_data", LoadDataCallback, request);
    }

    private void LoadDataCallback(LoadDataResponse response)
    {
        // TODO status, message
    }

    public void SaveData(bool useCurrentName = true, string filename = null)
    {
        var strWrp = new String();
        strWrp.data = filename;
        var request = new SaveDataRequest(useCurrentName, strWrp);
        rosSocket.CallService<SaveDataRequest, SaveDataResponse>("data_server/save_application_data", SaveDataCallback, request);
    }

    private void SaveDataCallback(SaveDataResponse response)
    {
        // TODO status, message
    }

    public void GetOptions()
    {
        var request = new GetOptionsRequest();
        rosSocket.CallService<GetOptionsRequest, GetOptionsResponse>("data_server/get_application_options", GetOptionsCallback, request);
    }

    private void GetOptionsCallback(GetOptionsResponse response)
    {
        // TODO options
    }

    /**
     * Program Interface
     */

    public Program program { get; private set; } = null;
    private VersionTag programVersion = null;
    private List<Dictionary<string,object>> programChangesManifest = new List<Dictionary<string,object>>();

    public bool hasLocalProgramChanges()
    {
        return programChangesManifest.Count > 0;
    }

    private void UpdateProgramDataCallback(UpdateData message)
    {
        //TODO need to do a smarter version of this where if tag match previous tag then only uses changes structure
        //program = Program.FromDict(message.data.data);
        //program.changesCallback = ProgramChangedCallback;
        programVersion = VersionTag.FromRos(message.currentTag);
    }

    public void ProgramGetData(bool all = true, string data = "{}")
    {
        var strWrp = new String();
        strWrp.data = data;
        var request = new GetDataRequest(all, strWrp);
        rosSocket.CallService<GetDataRequest, GetDataResponse>("data_server/get_program_data", ProgamGetDataCallback, request);
    }

    private void ProgamGetDataCallback(GetDataResponse response)
    {
        //TODO
    }

    public void ProgramSetData(string data = "{}", VersionTag tag = null)
    {
        if (tag == null)
        {
            tag = new VersionTag(SOURCE);
        }

        var strWrp = new String();
        strWrp.data = data;
        var request = new SetDataRequest(strWrp, tag.ToRos());
        rosSocket.CallService<SetDataRequest, SetDataResponse>("data_server/set_program_data", ProgramSetDataCallback, request);
    }

    private void ProgramSetDataCallback(SetDataResponse response)
    {
        //TODO
    }

    /**
     * Environment Interface
     */

    private void UpdateEnvironmentDataCallback(UpdateData message)
    {
        //TODO
    }

    private void EnvironmentGetData(bool all = true, string data = "{}")
    {
        var strWrp = new String();
        strWrp.data = data;
        var request = new GetDataRequest(all, strWrp);
        rosSocket.CallService<GetDataRequest, GetDataResponse>("data_server/get_environment_data", EnvironmentGetDataCallback, request);
    }

    private void EnvironmentGetDataCallback(GetDataResponse response)
    {
        //TODO
    }

    private void EnvironmentSetData(string data = "{}", VersionTag tag = null)
    {
        if (tag == null)
        {
            tag = new VersionTag(SOURCE);
        }

        var strWrp = new String();
        strWrp.data = data;
        var request = new SetDataRequest(strWrp, tag.ToRos());
        rosSocket.CallService<SetDataRequest, SetDataResponse>("data_server/set_environment_data", EnvironmentSetDataCallback, request);
    }

    private void EnvironmentSetDataCallback(SetDataResponse response)
    {
        //TODO
    }
}
