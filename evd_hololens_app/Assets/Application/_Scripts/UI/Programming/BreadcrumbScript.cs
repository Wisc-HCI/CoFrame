using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class BreadcrumbScript : MonoBehaviour
{
    /*
    public GameObject BreadcrumbItemPrefab;

    public GameObject Track;

    public float stepSize = 10.0f;
    public float displaceScalar = 0.001f;

    private NavigationScript navigation;
    private List<BreadcrumbItemScript> crumbs = new List<BreadcrumbItemScript>();

    public void Initialize(NavigationScript nav)
    {
        navigation = nav;
    }

    public void StateChange(string title, bool appendOnly = false)
    {
        // find where in path state exists
        int index = 0;
        bool found = false;
        for (int i = 0; i < crumbs.Count; i++)
        {
            if (crumbs[i].id == title)
            {
                index = i;
                found = true;
                continue;
            }
        }

        // remove all after found point
        if (!appendOnly)
        {
            var start = index + (found ? 1 : 0);
            var count = crumbs.Count - start;

            for (int i = start; i < crumbs.Count; i++)
            {
                Pop(i);
            }

            crumbs.RemoveRange(start, count);
        }

        // need to insert state after if not found
        if (!found)
        {
            Push(title);
        }
    }

    private void Push(string title)
    {
        var obj = Instantiate(BreadcrumbItemPrefab, Track.transform);

        var tf = obj.GetComponent<RectTransform>();
        float displace = crumbs.Count * (tf.rect.width + stepSize) * displaceScalar;
        tf.Translate(new Vector3(displace, 0, 0));

        var bs = obj.GetComponent<BreadcrumbItemScript>();
        bs.Initialize(title, title);
        bs.callback = Selected;

        crumbs.Add(bs);
    }

    private void Pop(int index)
    {
        Destroy(crumbs[index].gameObject);
    }

    private void Selected(string title)
    {
        navigation.BreadCrumbOptionSelected(title);
    }
    */
}
