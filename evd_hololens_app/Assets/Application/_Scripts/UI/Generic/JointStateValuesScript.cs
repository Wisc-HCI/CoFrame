using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class JointStateValuesScript : MonoBehaviour
{
    public GameObject container;
    public GameObject FloatSliderPrefab;
    public float OFFSET = 0.0f;
    public float SLOPE = 1.0f;
    public float UI_SCALAR = 1000.0f;

    [System.Serializable]
    public class Joint
    {
        public string name;
        public string units = "rad";
        public float minValue = 0.0f;
        public float maxValue = 2 * Mathf.PI;
        public float value = 0.0f;
    }

    public Joint[] joints;

    public Dictionary<string, FloatSliderScript> sliders = new Dictionary<string, FloatSliderScript>();

    private void Awake()
    {
        int count = 0;

        foreach (var j in joints)
        {
            var obj = Instantiate(FloatSliderPrefab,container.transform);
            obj.transform.Translate(new Vector3(0,count * SLOPE + OFFSET,0));

            var fs = obj.GetComponent<FloatSliderScript>();
            fs.units = j.units;
            fs.minValue = j.minValue;
            fs.maxValue = j.maxValue;
            fs.value = j.value;
            fs.name = j.name;

            sliders.Add(j.name, fs);
            count++;
        }

        GetComponent<RectTransform>().SetSizeWithCurrentAnchors(RectTransform.Axis.Vertical, UI_SCALAR * Mathf.Abs(count * SLOPE + OFFSET));
    }

    public float[] values
    {
        get
        {
            float[] v = new float[joints.Length];
            for(int i = 0; i < joints.Length; i++)
            {
                v[i] = sliders[joints[i].name].value;
            }
            return v;
        }

        set
        {
            for(int i = 0; i < joints.Length; i++)
            {
                sliders[joints[i].name].value = value[i];
            }
        }
    }

    public float[] defaultValues
    {
        get
        {
            float[] fv = new float[joints.Length];
            for (int i=0; i<joints.Length; i++)
            {
                fv[i] = joints[i].value;
            }
            return fv;
        }
    }
}
