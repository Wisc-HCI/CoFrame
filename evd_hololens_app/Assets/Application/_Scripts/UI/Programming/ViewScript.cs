using System.Collections.Generic;
using UnityEngine;

public class ViewScript : MonoBehaviour
{
    /*
    public ApplicationDataScript app;
    public NavigationScript navigation;
    public SceneScript scene;
    public string activeUuid;

    public List<System.Action<bool>> visibleCallbacks = new List<System.Action<bool>>();

    private bool updateNavigation = false;
    private string updateNavState = null;
    private string updateNavUuid = null;

    private bool disableNavigation = false;

    public void Initialize(ApplicationDataScript app, NavigationScript nav)
    {
        this.app = app;
        navigation = nav;
    }

    public void Visible(bool val, string uuid )
    {
        if (val != gameObject.activeSelf)
        {
            disableNavigation = false;
            updateNavigation = false;
            updateNavState = null;
            updateNavUuid = null;
        }

        gameObject.SetActive(val);
        activeUuid = uuid;

        foreach (var cb in visibleCallbacks)
        {
            cb(val);
        }
    }

    public void UpdateNavigation(string state, string uuid)
    {
        disableNavigation = false;
        updateNavigation = true;
        updateNavState = state;
        updateNavUuid = uuid;
    }

    public void DisableNavigation()
    {
        disableNavigation = true;
        updateNavigation = false;
        updateNavState = null;
        updateNavUuid = null;
    }

    private void Update()
    {
        if (disableNavigation)
        {
            disableNavigation = false;
            navigation.DisableParameterizationContext();
        }

        if (updateNavigation)
        {
            updateNavigation = false;
            navigation.SetParameterizationContext(updateNavState, updateNavUuid);
        }
    }
    */
}
