using System;
using UnityEngine;

namespace EvD
{
    public abstract class AbstractMarker : MonoBehaviour
    {
        public float OPAQUE = 1.0f;
        public float TRANSLUCENT = 0.5f;

        [SerializeField]
        private string _uuid = "";
        [SerializeField]
        private string _name = "";
        [SerializeField]
        private bool _isNameVisible = true;
        [SerializeField]
        private bool _allowNameInteraction = true;
        [SerializeField]
        private MarkerMeshTypeScript.Options _markerType = MarkerMeshTypeScript.Options.Location;

        public Action<AbstractMarker, bool> selectedCallback = null;

        public Transform objectTf = null;

        private bool _isBeingManipulated = false;

        private Renderer markerRenderer;
        private ParticleSystem particles;
        private TagNameEntryScript nameTag;
        private MarkerMeshTypeScript markerTypeScript;
        private CustomArrowTrajectoryPointScript trajPointScript;

        private Waypoint _waypoint = null;

        protected bool _visibility = true;

        private void Awake()
        {
            markerRenderer = GetComponentInChildren<Renderer>();

            particles = GetComponentInChildren<ParticleSystem>();
            StopParticles();

            markerTypeScript = GetComponentInChildren<MarkerMeshTypeScript>();
            markerTypeScript.SetMeshType(markerType);

            nameTag = GetComponentInChildren<TagNameEntryScript>();
            nameTag.nameUpdatedCallback = OnNameUpdated;
            nameTag.isInteractable = allowNameInteraction;
            nameTag.isVisible = isNameVisible;
            nameTag.name = name;

            if (markerType == MarkerMeshTypeScript.Options.Location)
            {
                nameTag.EMPTY_DISPLAY_NAME = "<Location>";
            }
            else if (markerType == MarkerMeshTypeScript.Options.Waypoint)
            {
                nameTag.EMPTY_DISPLAY_NAME = "<Waypoint>";
            }

            trajPointScript = GetComponent<CustomArrowTrajectoryPointScript>();
            trajPointScript.Instantiate(uuid, objectTf);

            MakeOpaque();
        }

        public bool isNameVisible
        {
            get
            {
                return _isNameVisible;
            }

            set
            {
                if (_isNameVisible != value)
                {
                    _isNameVisible = value;
                    nameTag.isVisible = _isNameVisible;
                }
            }
        }

        public bool allowNameInteraction
        {
            get
            {
                return _allowNameInteraction;
            }

            set
            {
                if (_allowNameInteraction != value)
                {
                    _allowNameInteraction = value;
                    nameTag.isInteractable = _allowNameInteraction;
                }
            }
        }

        public string uuid
        {
            get
            {
                return _uuid;
            }

            set
            {
                _uuid = value;
                trajPointScript.uuid = _uuid;
            }
        }

        public new string name
        {
            get
            {
                return _name;
            }

            set
            {
                if (_name != value)
                {
                    _name = value;
                    nameTag.name = _name;
                }
            }
        }

        public MarkerMeshTypeScript.Options markerType
        {
            get
            {
                return _markerType;
            }

            set
            {
                if (_markerType != value)
                {
                    _markerType = value;
                    markerTypeScript.SetMeshType(_markerType);

                    if (markerType == MarkerMeshTypeScript.Options.Location)
                    {
                        nameTag.EMPTY_DISPLAY_NAME = "<Location>";
                    }
                    else if (markerType == MarkerMeshTypeScript.Options.Waypoint)
                    {
                        nameTag.EMPTY_DISPLAY_NAME = "<Waypoint>";
                    }
                }
            }
        }

        public void OnNameUpdated(string name)
        {
            if (waypoint != null)
            {
                waypoint.name = name;
            }
        }

        public void MakeOpaque()
        {
            var c = markerRenderer.material.color;
            c.a = OPAQUE;
            markerRenderer.material.color = c;
        }

        public void MakeTranslucent()
        {
            var c = markerRenderer.material.color;
            c.a = TRANSLUCENT;
            markerRenderer.material.color = c;
        }

        public void StartParticles()
        {
            particles.Play();
        }

        public void StopParticles()
        {
            particles.Stop();
            particles.Clear();
        }

        public Vector3 Position
        {
            get
            {
                return objectTf.localPosition;
            }

            set
            {
                objectTf.localPosition = value;
            }
        }

        public Quaternion Orientation {
            get {
                return objectTf.localRotation;
            }

            set
            {
                objectTf.localRotation = value;
            }
        }

        public bool visibility
        {
            get
            {
                return _visibility;
            }

            set
            {
                if (_visibility != value)
                {
                    _visibility = value;
                    objectTf.gameObject.SetActive(_visibility);
                }
            }
        }

        public Waypoint waypoint
        {
            get
            {
                return _waypoint;
            }

            set
            {
                if (_waypoint != value)
                {
                    _waypoint = value;
                    uuid = _waypoint.uuid;
                    Position = _waypoint.position.ToUnity();
                    Orientation = _waypoint.orientation.ToUnity();
                    name = _waypoint.name;

                    string type = UtilityFunctions.GetExactType(_waypoint);
                    if (type == "location")
                    {
                        markerType = MarkerMeshTypeScript.Options.Location;
                    }
                    else if (type == "waypoint")
                    {
                        markerType = MarkerMeshTypeScript.Options.Waypoint;
                    }
                }
            }
        }

        public virtual bool isBeingManipulated
        {
            get
            {
                return _isBeingManipulated;
            }

            protected set
            {
                if (_isBeingManipulated != value)
                {
                    _isBeingManipulated = value;
                }
            }
        }
    }
}


