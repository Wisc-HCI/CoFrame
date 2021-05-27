using System.Collections.Generic;
using UnityEngine;
using EvD;

public class ProgramRootScript : MonoBehaviour
{
    /*
    public ViewScript view;

    public GameObject primitivePrefab;

    public GameObject primitiveTrack;

    public int maxPrimitives = 9;
    public float stepSize = 10.0f;
    public float displaceScalar = 0.001f;

    public OkayPopupMessageScript warningPanel;

    private string activePrimitive = null;
    private List<PrimitiveScript> primitives = new List<PrimitiveScript>();

    private void Awake()
    {
        view = GetComponent<ViewScript>();
        view.visibleCallbacks.Add(Visible);
    }

    public void Visible(bool val)
    {
        if (val)
        {
            // TODO load primitives from app. Clear previous list

            OnSelect(activePrimitive);
        }
    }

    public void Add(string type)
    {
        if (primitives.Count + 1 > maxPrimitives)
        {
            warningPanel.DisplayMessage("Reached maximum number of primitives.", title: "Warning");
        }
        else
        {
            // create new primitive
            Primitive primitive;
            if (type == "move")
            {
                primitive = new MovePrimitive();
            }
            else if (type == "delay")
            {
                primitive = new DelayPrimitive();
            }
            else if (type == "grip")
            {
                primitive = new GripPrimitive();
            }
            else if (type == "machine")
            {
                primitive = new MachinePrimitive();
            }
            else
            {
                primitive = new Primitive(type);
            }

            var p = Instantiate(primitivePrefab, primitiveTrack.transform, false);

            var tf = p.GetComponent<RectTransform>();
            float displace = primitives.Count * (tf.rect.width + stepSize) * displaceScalar;
            tf.Translate(new Vector3(displace, 0, 0));

            var pscript = p.GetComponent<PrimitiveScript>();
            pscript.primitiveClickCallback = PrimitiveSelectedCallback;
            pscript.deleteClickCallback = OnDeleteClick;
            pscript.shiftLeftClickCallback = OnShiftLeftClick;
            pscript.shiftRightClickCallback = OnShiftRightClick;
            pscript.OnInstantiate(type, primitive.uuid, primitive.name);

            primitives.Add(pscript);
            view.app.AddPrimitive(primitive.uuid, primitive);

            // set as active
            OnSelect(primitive.uuid);
        }
    }

    public void PrimitiveSelectedCallback(string uuid)
    {
        if (uuid != activePrimitive)
        {
            OnSelect(uuid);
        }
        else
        {
            view.navigation.ParameterizeButtonClicked();
        }
    }

    public void OnSelect(string uuid)
    {
        activePrimitive = uuid;
        string type = null;
        for (int i = 0; i < primitives.Count; i++)
        {
            bool match = primitives[i].uuid == uuid;
            primitives[i].ActivePrimitive(match);
            if (match)
            {
                type = primitives[i].type;
            }
        }

        if (type != null)
        {
            view.UpdateNavigation(type, uuid);
        }
        else
        {
            view.DisableNavigation();
        }
    }

    public void OnDeleteClick(string uuid)
    {
        // find location of primitive
        int index = FindPrimitiveIndex(uuid);
        if (index > -1)
        {
            view.app.DeletePrimitive(uuid);

            // shift back
            Destroy(primitives[index].gameObject);
            primitives.RemoveAt(index);
            Shift(index, primitives.Count, -1);

            // find next active
            if (index - 1 < primitives.Count && index - 1 >= 0)
            {
                OnSelect(primitives[index - 1].uuid);
            }
            else if (index - 1 < 0 && primitives.Count >= 1)
            {
                OnSelect(primitives[0].uuid);
            }
            else
            {
                OnSelect(null);
            }
        }
    }

    private void Shift(int start, int end, int direction)
    {
        for (int j = start; j < end; j++)
        {
            var tf = primitives[j].GetComponent<RectTransform>();
            float displace = direction * (tf.rect.width + stepSize) * displaceScalar;
            tf.Translate(new Vector3(displace, 0, 0));
        }
    }

    public void OnShiftLeftClick(string uuid)
    {
        int index = FindPrimitiveIndex(uuid);

        if (index - 1 < 0)
        {
            warningPanel.DisplayMessage("At start of program.", title: "Warning");
        }
        else
        {
            Shift(index, index + 1, -1);
            Shift(index - 1, index, 1);

            var obj = primitives[index];
            primitives[index] = primitives[index - 1];
            primitives[index - 1] = obj;

            view.app.UpdatePrimitiveOrder(GetAllUuids());
        }
    }

    public void OnShiftRightClick(string uuid)
    {
        int index = FindPrimitiveIndex(uuid);

        if (index + 1 >= primitives.Count)
        {
            warningPanel.DisplayMessage("At end of program.", title: "Warning");
        }
        else
        {
            Shift(index, index + 1, 1);
            Shift(index + 1, index + 2, -1);

            var obj = primitives[index];
            primitives[index] = primitives[index + 1];
            primitives[index + 1] = obj;

            view.app.UpdatePrimitiveOrder(GetAllUuids());
        }
    }

    private int FindPrimitiveIndex(string uuid)
    {
        int index = -1;
        for (int i = 0; i < primitives.Count; i++)
        {
            if (primitives[i].uuid == uuid)
            {
                index = i;
                break;
            }
        }
        return index;
    }

    private List<string> GetAllUuids()
    {
        List<string> uuids = new List<string>();
        foreach (var p in primitives)
        {
            uuids.Add(p.uuid);
        }
        return uuids;
    }
    */
}
