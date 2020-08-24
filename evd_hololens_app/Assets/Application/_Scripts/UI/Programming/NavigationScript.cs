using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class NavigationScript : MonoBehaviour
{
    /*
    [System.Serializable]
    public struct ViewEntry
    {
        public string name;
        public ViewScript view;
        public string[] children;
        public string parent;
    }

    public ApplicationDataScript app;

    public ViewEntry[] viewList;

    public GameObject backGrayout;
    public GameObject setupGrayout;
    public GameObject programGrayout;
    public GameObject operateGrayout;

    public BreadcrumbScript breadcrumb;

    private string paramNextState = null;
    private string paramUuid = null;

    private string primaryState = null;

    private Dictionary<string, string> uuids = new Dictionary<string, string>();
    private Dictionary<string, ViewScript> views = new Dictionary<string, ViewScript>();
    private Dictionary<string, string[]> children = new Dictionary<string, string[]>();
    private Dictionary<string, string> parents = new Dictionary<string, string>();

    public void Initialize(ApplicationDataScript app)
    {
        this.app = app;
        breadcrumb.Initialize(this);

        foreach (var entry in viewList)
        {
            entry.view.Initialize(app, this);
            uuids.Add(entry.name, null);
            views.Add(entry.name, entry.view);
            children.Add(entry.name, entry.children);
            parents.Add(entry.name, entry.parent);
        }

        OperateButtonClicked();
    }

    public void SetupButtonClicked()
    {
        SetViewState("setup", uuids["setup"]);
        setupGrayout.SetActive(true);
        programGrayout.SetActive(false);
        operateGrayout.SetActive(false);
    }

    public void ProgramButtonClicked()
    {
        SetViewState("program", uuids["program"]);
        setupGrayout.SetActive(false);
        programGrayout.SetActive(true);
        operateGrayout.SetActive(false);
    }

    public void OperateButtonClicked()
    {
        SetViewState("operate", uuids["operate"]);
        setupGrayout.SetActive(false);
        programGrayout.SetActive(false);
        operateGrayout.SetActive(true);
    }

    public void BackButtonClicked()
    {
        var parentId = parents[primaryState];
        SetViewState(parentId, uuids[parentId]);
    }

    public void BreadCrumbOptionSelected(string state)
    {
        SetViewState(state, uuids[state]);
    }

    public void SetViewState(string state, string uuid, bool appendCrumb = false)
    {
        app.scene.SetScene(state, uuid);
        primaryState = state;
        uuids[primaryState] = uuid;
        foreach (var key in views.Keys)
        {
            if (key == primaryState)
            {
                views[key].Visible(true, uuid);
                backGrayout.SetActive(parents[key] == "");
                breadcrumb.StateChange(state, appendCrumb);
            }
            else
            {
                views[key].Visible(false, null);
            }
        }
    }

    public void ParameterizeButtonClicked()
    {
        SetViewState(paramNextState, paramUuid, true);
        paramNextState = null;
        paramUuid = null;
    }

    public void SetParameterizationContext(string nextState, string uuid)
    {
        paramNextState = nextState;
        paramUuid = uuid;
    }

    public void DisableParameterizationContext()
    {
        paramNextState = null;
        paramUuid = null;
    }
    */
}
