using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Cobots
{
    [System.Serializable]
    public class Task : Primitive
    {

        /*
         * Private Members
         */

        private List<Primitive> _primitives = null;
        private Context _context = null;

        /*
        * Constructors
        */

        public Task(List<Primitive> primitives = null, string type = "", string name = "", 
                    string uuid = null, Node parent = null, bool appendType = true, 
                    Context context = null) 
        : base(appendType ? "task." + type : type, name, uuid, parent, appendType)
        {
            this.primitives = primitives != null ? primitives : new List<Primitive>();
            this.context = context != null ? context : new Context();
        }

        public new static Task FromDict(Dictionary<string, object> dct)
        {
            var primitives = new List<Primitive>();
            foreach (var p in (List<Dictionary<string,object>>)dct["primitives"])
            {
                primitives.Add(Primitive.FromDict(p));
            }

            return new Task(
                primitives: primitives,
                context: Context.FromDict((Dictionary<string,object>)dct["context"]),
                type: (string)dct["type"],
                name: (string)dct["name"],
                uuid: (string)dct["uuid"],
                appendType: false
            );
        }

        public override Dictionary<string, object> ToDict()
        {
            var prmDct = new List<Dictionary<string, object>>();
            foreach (var p in primitives)
            {
                prmDct.Add(p.ToDict());
            }

            var dct = base.ToDict();
            dct.Add("primitives", prmDct);
            dct.Add("context", context.ToDict());
            return dct;
        }

        /*
         * Accessors and Modifiers
         */

        public override Context context
        {
            get
            {
                return _context;
            }

            set
            {
                if (_context != value)
                {
                    if (_context != null)
                    {
                        _context.RemoveFromCache();
                    }

                    _context = value;
                    if (_context != null)
                    {
                        _context.parent = this;
                        if (parent != null)
                        {
                            _context.parentContext = parent.context;
                        }
                    }

                    UpdatedAttribute("context", "set");
                }
            }
        }

        public List<Primitive> primitives
        {
            get
            {
                return _primitives;
            }

            set
            {
                if (_primitives != value)
                {
                    if (_primitives != null)
                    {
                        foreach (var p in _primitives)
                        {
                            p.RemoveFromCache();
                        }
                    }

                    _primitives = value;
                    foreach (var p in _primitives)
                    {
                        p.parent = this;
                    }

                    UpdatedAttribute("primitives", "set");
                }
            }
        }

        public void AddPrimitive(Primitive prm)
        {
            prm.parent = this;
            primitives.Add(prm);
            UpdatedAttribute("primitives", "add", prm.uuid);
        }

        public void InsertPrimitive(int idx, Primitive prm)
        {
            prm.parent = this;
            primitives.Insert(idx, prm);
            UpdatedAttribute("primitives", "add", prm.uuid);
        }

        public Primitive GetPrimitive(string uuid)
        {
            return primitives[SearchPrimtivesForIndex(uuid)];
        }

        public void DeletePrimitive(string uuid)
        {
            var idx = SearchPrimtivesForIndex(uuid);
            primitives[idx].RemoveFromCache();
            primitives.RemoveAt(idx);
            UpdatedAttribute("primitives", "delete", uuid);
        }

        public void ReorderPrimitives(string uuid, int shift)
        {
            int baseIdx = SearchPrimtivesForIndex(uuid);
            int newIdx = baseIdx + shift;
            if (newIdx < 0 || newIdx >= primitives.Count)
            {
                throw new System.Exception("Outside of primitives list");
            }

            var copy = primitives[baseIdx];
            primitives.RemoveAt(baseIdx);
            primitives.Insert(newIdx, copy);
            UpdatedAttribute("primitives", "reorder");
        }

        public int SearchPrimtivesForIndex(string uuid)
        {
            int idx = -1;
            for (int i=0; i<primitives.Count; i++)
            {
                if (primitives[i].uuid == uuid)
                {
                    idx = i;
                    break;
                }
            }
            return idx;
        }

        public override void Set(Dictionary<string, object> dct)
        {
            
            if (dct.ContainsKey("primitives"))
            {
                var prms = new List<Primitive>();
                foreach (var p in (List<Dictionary<string, object>>)dct["primitives"])
                {
                    prms.Add(Primitive.FromDict(p));
                }
                primitives = prms;
            }

            if (dct.ContainsKey("context"))
            {
                context = Context.FromDict((Dictionary<string, object>)dct["context"]);
            }

            base.Set(dct);
        }

        /*
         * Cache Methods
         */

        public override void RemoveFromCache()
        {
            foreach (var p in primitives)
            {
                p.RemoveFromCache();
            }

            context.RemoveFromCache();

            base.RemoveFromCache();
        }

        public override void AddToCache()
        {
            foreach (var p in primitives)
            {
                p.AddToCache();
            }

            context.AddToCache();

            base.AddToCache();
        }

        /*
         * Children Methods
         */

        public override bool DeleteChild(string uuid)
        {
            bool success = false;

            if (uuid == context.uuid)
            {
                context = new Context();
                success = true;
            }
            else
            {
                foreach (var p in primitives)
                {
                    if (p.uuid == uuid)
                    {
                        DeletePrimitive(uuid);
                        success = true;
                        break;
                    }
                }
            }

            return success;
        }

        /*
         * Update Methods
         */

        public override void DeepUpdate()
        {
            foreach (var p in primitives)
            {
                p.DeepUpdate();
            }

            context.DeepUpdate();

            base.DeepUpdate();

            UpdatedAttribute("primitives", "update");
            UpdatedAttribute("context", "update");
        }

        public override void ShallowUpdate()
        {
            base.ShallowUpdate();

            UpdatedAttribute("primitives", "update");
            UpdatedAttribute("context", "update");
        }

    }

    [System.Serializable]
    public class CloseGripper : Task
    {
        public CloseGripper(float position=100, float effort=100, float speed=100, string type = "", string name = "",
                            string uuid = null, Node parent = null, bool appendType = true,
                            List <Primitive> primitives = null, Context context = null) 
        : base(primitives, appendType ? "close-gripper." + type : type, name, uuid, parent, appendType, context)
        {
            if (primitives.Count == 0)
            {
                AddPrimitive(new Gripper(
                    position: position,
                    effort: effort,
                    speed: speed
                ));
            }
        }
    }

    [System.Serializable]
    public class OpenGripper : Task
    {
        public OpenGripper(float position=0, float effort=100, float speed=100, string type = "", string name = "",
                           string uuid = null, Node parent = null, bool appendType = true,
                           List<Primitive> primitives = null, Context context = null)
        : base(primitives, appendType ? "open-gripper." + type : type, name, uuid, parent, appendType, context)
        {
            if (primitives.Count == 0)
            {
                AddPrimitive(new Gripper(
                    position: position,
                    effort: effort,
                    speed: speed
                ));
            }
        }
    }

    [System.Serializable]
    public class PickAndPlace : Task
    {
        public PickAndPlace(string startLocUuid = null, string pickLocUuid = null, string placeLocUuid = null, 
                            string type = "", string name = "", string uuid = null, Node parent = null, 
                            bool appendType = true, List<Primitive> primitives = null, Context context = null)
        : base(primitives, appendType ? "pick-and-place." + type : type, name, uuid, parent, appendType, context)
        { 
            if (primitives.Count == 0)
            {
                AddPrimitive(new MoveTrajectory(startLocUuid,pickLocUuid));
                AddPrimitive(new CloseGripper());
                AddPrimitive(new MoveTrajectory(pickLocUuid,placeLocUuid));
                AddPrimitive(new OpenGripper());
            }
        }
    }

    [System.Serializable]
    public class Retract : MoveTrajectory
    {
        public Retract(string startLocUuid, string endLocUuid, List<Trajectory> trajectories = null,
                       string runnableTrajUuid = null, string type = "", string name = "",
                       string uuid = null, Node parent = null, bool appendType = true,
                       bool createDefault = true)
        : base(startLocUuid, endLocUuid, trajectories, runnableTrajUuid, appendType ? "retract." + type : type, 
               name, uuid, parent, appendType, createDefault) {}
    }

    [System.Serializable]
    public class Initialize : Task
    {
        public Initialize(string homeLocUuid = null, string machineUuid = null, string type = "", string name = "", 
                          string uuid = null, Node parent = null, bool appendType = true, List<Primitive> primitives = null, 
                          Context context = null) 
        : base(primitives, appendType ? "initialize." + type : type, name, uuid, parent, appendType, context)
        {
            if (primitives.Count == 0)
            {
                AddPrimitive(new MoveUnplanned(homeLocUuid, true));
                AddPrimitive(new OpenGripper());
                AddPrimitive(new MachineInitialize(machineUuid));
            }
        }
    }

    [System.Serializable]
    public class MachineBlockingProcess : Task
    {
        public MachineBlockingProcess(string machineUuid = null, string type = "", string name = "", string uuid = null, 
                                      Node parent = null, bool appendType = true, List<Primitive> primitives = null, 
                                      Context context = null) 
        : base(primitives, appendType ? "machine-blocking-process." + type : type, name, uuid, parent, appendType, context)
        {
            if (primitives.Count == 0)
            {
                AddPrimitive(new MachineStart(machineUuid));
                AddPrimitive(new MachineWait(machineUuid));
                AddPrimitive(new MachineStop(machineUuid));
            }
        }
    }

    [System.Serializable]
    public class Loop : Task
    {
        public Loop(List<Primitive> primitives = null, string type = "", string name = "",
                    string uuid = null, Node parent = null, bool appendType = true,
                    Context context = null) 
        : base(primitives, appendType ? "loop." + type : type, name, uuid, parent, appendType, context) {}
    }
}
