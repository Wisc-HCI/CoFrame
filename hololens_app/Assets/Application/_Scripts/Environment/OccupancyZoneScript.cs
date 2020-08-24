using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class OccupancyZoneScript : MonoBehaviour
{

    [SerializeField]
    protected float DEFAULT_GROUND_HEIGHT = -0.62f;

    private Cobots.OccupancyZone _model = null;
    private string _uuid;

    private void Awake()
    {
        SetGroundHeight(DEFAULT_GROUND_HEIGHT);
    }

    public string uuid
    {
        get
        {
            return _uuid;
        }

        private set
        {
            if (_uuid != value)
            {
                _uuid = value;
            }
        }
    }

    public Cobots.OccupancyZone model
    {
        get
        {
            return _model;
        }

        set
        {
            if (value != null)
            {
                if (_model != value)
                {
                    _model = value;
                    uuid = _model.uuid;
                    position = new Vector2(_model.positionX, _model.positionZ);
                    scale = new Vector2(_model.scaleX, _model.scaleZ);
                }
            }
            else
            {
                _model = null;
                uuid = null;
                position = new Vector2();
                scale = new Vector2();
            }
        }
    }

    public virtual Vector2 position
    {
        get
        {
            return new Vector2(transform.localPosition.x, transform.localPosition.z);
        }

        protected set
        {
            transform.localPosition = new Vector3(value.x, transform.localPosition.y, value.y);
        }
    }

    public virtual void SetPosition(Vector2 value)
    {
        if (model != null)
        {
            model.positionX = value.x;
            model.positionZ = value.y;
        }
        position = value;
    }

    public virtual Vector2 scale
    {
        get
        {
            return new Vector2(transform.localScale.x, transform.localScale.z);
        }

        protected set
        {
            transform.localScale = new Vector3(value.x, transform.localScale.y, value.y);
        }
    }

    public virtual void SetScale(Vector2 value)
    {
        if (model != null)
        {
            model.scaleX = value.x;
            model.scaleZ = value.y;
        }
        scale = value;
    }

    public virtual void SetGroundHeight(float y)
    {
        transform.localPosition = new Vector3(transform.localPosition.x, y, transform.localPosition.z);
    }
}
