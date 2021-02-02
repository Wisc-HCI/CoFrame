using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class HumanOccupancyZoneScript : OccupancyZoneScript
{
    //public GameObject BasePlate;
    //public GameObject LeftTab;
    //public GameObject RightTab;
    //public GameObject FrontTab;
    //public GameObject BackTab;

    public System.Action<string, Vector2, Vector2> manipulatedCallback = null; // uuid, position, scale

    [SerializeField]
    private bool _visible = false;
    [SerializeField]
    private bool _controlsVisible = false;
    private Dictionary<string, bool> objectBeingManipulated = new Dictionary<string, bool>();

    private void Awake()
    {
        objectBeingManipulated.Add("base", false);
        objectBeingManipulated.Add("left", false);
        objectBeingManipulated.Add("right", false);
        objectBeingManipulated.Add("front", false);
        objectBeingManipulated.Add("back", false);

        position = new Vector2(0, 0);

        SetGroundHeight(DEFAULT_GROUND_HEIGHT);

        /*
        BasePlate.SetActive(_visible);
        bool setControls = _visible && controlsVisible;
        LeftTab.SetActive(setControls);
        RightTab.SetActive(setControls);
        FrontTab.SetActive(setControls);
        BackTab.SetActive(setControls);
        */
        //BasePlate.GetComponent<ObjectManipulator>().enabled = _controlsVisible;
    }

    private void Update()
    {
        /*
        bool beingManipulated = false;

        // scan through manipulated to adjust baseplate size / position + tab positions
        if (objectBeingManipulated["base"])
        {
            beingManipulated = true;
            var h = BasePlate.transform.localPosition.y;
            var w = BasePlate.transform.localScale.x;
            var l = BasePlate.transform.localScale.z;

            var x = BasePlate.transform.localPosition.x;
            var z = BasePlate.transform.localPosition.z;

            SetTabPosition(x, z, w, h, l);
        }

        if (objectBeingManipulated["left"] || objectBeingManipulated["right"])
        {
            beingManipulated = true;
            var left = LeftTab.transform.localPosition;
            var right = RightTab.transform.localPosition;

            var dif = right - left;
            
            if (dif.x < 0) // flipped
            {
                var temp = RightTab;
                RightTab = LeftTab;
                LeftTab = temp;
            }

            // rescale
            var prevScale = BasePlate.transform.localScale;
            BasePlate.transform.localScale = new Vector3(Mathf.Abs(dif.x), prevScale.y, prevScale.z);

            // move tabs
            var h = BasePlate.transform.localPosition.y;
            var w = BasePlate.transform.localScale.x;
            var l = BasePlate.transform.localScale.z;

            var x = BasePlate.transform.localPosition.x;
            var z = BasePlate.transform.localPosition.z;

            SetTabPosition(x, z, w, h, l);
        }

        if (objectBeingManipulated["front"] || objectBeingManipulated["back"])
        {
            beingManipulated = true;

            var front = FrontTab.transform.localPosition;
            var back = BackTab.transform.localPosition;

            var dif = back - front;

            if (dif.z < 0) // flipped
            {
                var temp = FrontTab;
                FrontTab = BackTab;
                BackTab = temp;
            }

            // rescale
            var prevScale = BasePlate.transform.localScale;
            BasePlate.transform.localScale = new Vector3(prevScale.x, prevScale.y, Mathf.Abs(dif.z));

            // move tabs
            var h = BasePlate.transform.localPosition.y;
            var w = BasePlate.transform.localScale.x;
            var l = BasePlate.transform.localScale.z;

            var x = BasePlate.transform.localPosition.x;
            var z = BasePlate.transform.localPosition.z;

            SetTabPosition(x, z, w, h, l);            
        }

        if (beingManipulated)
        {
            model.positionX = position.x;
            model.positionZ = position.y;
            model.scaleX = scale.x;
            model.scaleZ = scale.y;

            manipulatedCallback?.Invoke(uuid, position, scale);
        }
        */
    }

    public override Vector2 position
    {
        get
        {
            return new Vector2();
            //return new Vector2(BasePlate.transform.localPosition.x, BasePlate.transform.localPosition.z);
        }

        protected set
        {
            /*
            bool beingManipulated = false;
            foreach (var e in objectBeingManipulated.Values)
            {
                beingManipulated = beingManipulated || e;
            }

            if (!beingManipulated)
            {
                var h = BasePlate.transform.localPosition.y;
                // set base
                BasePlate.transform.localPosition = new Vector3(value.x, h, value.y);

                // set scale tabs
                var w = BasePlate.transform.localScale.x;
                var l = BasePlate.transform.localScale.z;

                SetTabPosition(value.x, value.y, w, h, l);
            }
            */
        }
    }

    public override Vector2 scale
    {
        get
        {
            return new Vector2();
            //return new Vector2(BasePlate.transform.localScale.x, BasePlate.transform.localScale.z);
        }

        protected set
        {
            /*
            bool beingManipulated = false;
            foreach (var e in objectBeingManipulated.Values)
            {
                beingManipulated = beingManipulated || e;
            }

            if (!beingManipulated)
            {
                BasePlate.transform.localScale = new Vector3(value.x, BasePlate.transform.localScale.y, value.y);

                var h = BasePlate.transform.localPosition.y;
                var w = BasePlate.transform.localScale.x;
                var l = BasePlate.transform.localScale.z;

                SetTabPosition(BasePlate.transform.localPosition.x, BasePlate.transform.localPosition.z, w, h, l);
            }
            */
        }
    }

    public bool visible
    {
        get
        {
            return _visible;
        }

        set
        {
            if (_visible != value)
            {
                _visible = value;
                /*
                BasePlate.SetActive(_visible);

                bool setControls = _visible && controlsVisible;
                LeftTab.SetActive(setControls);
                RightTab.SetActive(setControls);
                FrontTab.SetActive(setControls);
                BackTab.SetActive(setControls);
                */
            }
        }
    }

    public bool controlsVisible
    {
        get
        {
            return _controlsVisible;
        }

        set
        {
            if (_controlsVisible != value)
            {
                _controlsVisible = value;

                if (visible)
                {
                    /*
                    LeftTab.SetActive(_controlsVisible);
                    RightTab.SetActive(_controlsVisible);
                    FrontTab.SetActive(_controlsVisible);
                    BackTab.SetActive(_controlsVisible);
                    */
                }

                //BasePlate.GetComponent<ObjectManipulator>().enabled = _controlsVisible;
            }
        }
    }

    public override void SetGroundHeight(float y)
    {
        /*
        BasePlate.transform.localPosition = new Vector3(BasePlate.transform.localPosition.x, y, base.transform.localPosition.z);

        // move tabs
        var h = BasePlate.transform.localPosition.y;
        var w = BasePlate.transform.localScale.x;
        var l = BasePlate.transform.localScale.z;

        var x = BasePlate.transform.localPosition.x;
        var z = BasePlate.transform.localPosition.z;

        SetTabPosition(x, z, w, h, l);
        */
    }

    public void OnBaseplateManipulationStarted()
    {
        objectBeingManipulated["base"] = true;
    }

    public void OnBaseplateManipulationEnded()
    {
        objectBeingManipulated["base"] = false;
    }

    public void OnScaleTabStarted(string id)
    {
        objectBeingManipulated[id] = true;
    }

    public void OnScaleTabEnded(string id)
    {
        objectBeingManipulated[id] = false;
    }

    private void SetTabPosition(float x, float z, float w, float h, float l)
    {
        /*
        LeftTab.transform.localPosition = new Vector3(x - w / 2, h, z);
        RightTab.transform.localPosition = new Vector3(x + w / 2, h, z);
        FrontTab.transform.localPosition = new Vector3(x, h, z - l / 2);
        BackTab.transform.localPosition = new Vector3(x, h, z + l / 2);
        */
    }
}
