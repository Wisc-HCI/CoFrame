using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TraceManagerScript : MonoBehaviour
{

    public GameObject TracePrefab;

    private System.Action<string, Vector3, Quaternion, int> _colliderClickCallback = null;

    private Dictionary<string, TraceRendererScript> traces = new Dictionary<string, TraceRendererScript>();

    public TraceRendererScript GetTrace(string uuid)
    {
        if (traces.ContainsKey(uuid))
        {
            return traces[uuid];
        }
        else
        {
            return null;
        }
    }

    public System.Action<string, Vector3, Quaternion, int> colliderClickCallback
    {
        private get
        {
            return _colliderClickCallback;
        }

        set
        {
            if (_colliderClickCallback != value)
            {
                _colliderClickCallback = value;
                foreach (var entry in traces)
                {
                    entry.Value.colliderClickCallback = _colliderClickCallback;
                }
            }
        }
    }

    public void OnTraceUpdate(Cobots.Trace trace)
    {
        var ts = traces[trace.uuid];
        ts.OnUpdate(trace);
    }

    public void OnTraceAdd(Cobots.Trace trace)
    {
        var obj = Instantiate(TracePrefab, transform, false);
        var ts = obj.GetComponent<TraceRendererScript>();
        ts.OnUpdate(trace);

        traces.Add(trace.uuid, ts);
    }

    public void OnTraceDelete(string uuid)
    {
        if (traces.ContainsKey(uuid))
        {
            traces[uuid].OnDelete();
            Destroy(traces[uuid].gameObject);
            traces.Remove(uuid);
        }
    }

}
