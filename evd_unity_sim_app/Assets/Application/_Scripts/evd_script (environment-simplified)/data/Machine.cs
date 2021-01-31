using System.Collections;
using System.Collections.Generic;


namespace EvD
{
    namespace Data
    {

        public class Machine : Node
        {

            /*
            * Private Attributes
            */

            private Dictionary<string,Region> _inputRegions = null;
            private Dictionary<string,Region> _outputRegions = null;
            private string _machineType;
            private MachineRecipe _recipe = null;

            /*
            * Constructors
            */

            public Machine(Dictionary<string,Region> inputRegions = null, Dictionary<string,Region> outputRegions, 
                           MachineRecipe recipe, string type = "", string name = "", string uuid = null, 
                           Node parent = null, bool appendType = true )
            : base(appendType ? "machine." + type : type, name, uuid, parent, appendType)
            {
                this.inputRegions = inputRegions;
                this.outputRegions = outputRegions;
                this.recipe = recipe;
            }

            public static new FromDict(Dictionary<string,object> dct)
            {
                return new MachineRecipe(
                    inputRegions: (Dictionary<string,int>)dct["input_regions"],
                    outputRegions: (Dictionary<string,int>)dct["output_regions"],
                    recipe: MachineRecipe.FromDict(dct["recipe"]),
                    type: dct.ContainsKey("type") ? (string)dct["type"] : "",
                    name: dct.ContainsKey("name") ? (string)dct["name"] : "",
                    uuid: dct.ContainsKey("uuid") ? (string)dct["uuid"] : null,
                    appendType: !dct.ContainsKey("type")
                );
            }

            public override Dictionary<string, object> ToDict()
            {
                Dictionary<string,object> inputDcts = new Dictionary<string, object>();
                foreach (KeyValuePair<string,Region> entry in inputRegions)
                {
                    inputDcts.Add(entry.Key,entry.Value.ToDict());
                } 

                Dictionary<string,object> outputDcts = new Dictionary<string, object>();
                foreach (KeyValuePair<string,Region> entry in outputRegions)
                {
                    outputDcts.Add(entry.Key,entry.Value.ToDict());
                } 

                var msg = base.ToDict();
                msg.Add("input_regions", inputDcts);
                msg.Add("output_regions",outputDcts);
                msg.Add("recipe",recipe.ToDict());
                return msg;
            }

            /*
            * Public Acessors Modifiers
            */

            public string machineType
            {
                get
                {
                    return _machineType;
                }
            }

            public Dictionary<string,Region> inputRegions
            {
                get
                {
                    return _inputRegions;
                }

                set
                {
                    if (_inputRegions != value)
                    {
                        if (_inputRegions != null)
                        {
                            foreach (var region in _inputRegions.Values())
                            {
                                region.RemoveFromCache();
                            }
                        }
                        
                        _inputRegions = value;
                        if (_inputRegions != null)
                        {
                            foreach (var region in _inputRegions.Values())
                            {
                                region.parent = this;
                            }
                        }

                        ComputeType();
                        UpdatedAttribute("input_regions","set");
                    }
                }
            }

            public void AddInputRegion(string thingType, Region region, bool allowUpdate = false)
            {
                string verb = "add";

                if (inputRegions.ContainsKey(thingType))
                {
                    if (!allowUpdate)
                    {
                        throw new System.Exception("Thing Region is about to be overrided, however override is not allowed");
                    }
                    else
                    {
                        verb = "update";
                        inputRegions[thingType] = region;
                    }
                }
                else
                {
                    inputRegions.Add(thingType,region);
                }

                ComputeType();
                UpdatedAttribute("input_regions",verb);
            }

            public void DeleteInputRegion(string thingType)
            {
                if (inputRegions.ContainsKey(thingType))
                {
                    throw new System.Exception("No such thing in Input Regions");
                }

                inputRegions[thingType].RemoveFromCache();
                inputRegions.Remove(thingType);
                ComputeType();
                UpdatedAttribute("input_regions","delete");
            }

            public Region GetInputRegion(string thingType)
            {
                return inputRegions[thingType];
            }

            public string FindInputRegionThingType(string regionUuid)
            {
                string type = null;

                foreach (KeyValuePair<string,Region> entry in inputRegions)
                {
                    if (entry.Value.uuid == regionUuid)
                    {
                        type = entry.Key;
                        break;
                    }
                }

                return type;
            }

            public Dictionary<string,Region> outputRegions
            {
                get
                {
                    return _outputRegions;
                }

                set
                {
                    if (_outputRegions != value)
                    {
                        if (_outputRegions != null)
                        {
                            foreach (var region in _outputRegions.Values())
                            {
                                region.RemoveFromCache();
                            }
                        }
                        
                        _outputRegions = value;
                        if (_outputRegions != null)
                        {
                            foreach (var region in _outputRegions.Values())
                            {
                                region.parent = this;
                            }
                        }

                        ComputeType();
                        UpdatedAttribute("output_regions","set");
                    }
                }
            }
        
            public void AddOutputRegion(string thingType, Region region, bool allowUpdate = false)
            {
                string verb = "add";

                if (outputRegions.ContainsKey(thingType))
                {
                    if (!allowUpdate)
                    {
                        throw new System.Exception("Thing Region is about to be overrided, however override is not allowed");
                    }
                    else
                    {
                        verb = "update";
                        outputRegions[thingType] = region;
                    }
                }
                else
                {
                    outputRegions.Add(thingType,region);
                }

                ComputeType();
                UpdatedAttribute("output_regions",verb);
            }

            public void DeleteOutputRegion(string thingType)
            {
                if (outputRegions.ContainsKey(thingType))
                {
                    throw new System.Exception("No such thing in Output Regions");
                }

                outputRegions[thingType].RemoveFromCache();
                outputRegions.Remove(thingType);
                ComputeType();
                UpdatedAttribute("output_regions","delete");
            }

            public Region GetOutputRegion(string thingType)
            {
                return outputRegions[thingType];
            }

            public string FindOutputRegionThingType(string regionUuid)
            {
                string type = null;

                foreach (KeyValuePair<string,Region> entry in outputRegions)
                {
                    if (entry.Value.uuid == regionUuid)
                    {
                        type = entry.Key;
                        break;
                    }
                }

                return type;
            }

            public MachineRecipe recipe
            {
                get
                {
                    return _recipe;
                }

                set
                {
                    if (_recipe != value)
                    {
                        if (value != null)
                        {
                            throw new System.Exception("Recipe cannot be null");
                        }

                        _recipe.RemoveFromCache();
                        _recipe = value;
                        _recipe.parent = this;
                        UpdatedAttribute("recipe","set");
                    }
                }
            }

            public override void Set(Dictionary<string, object> dct)
            {
                if (dct.ContainsKey("input_regions"))
                {
                    Dictionary<string,Region> regions = new Dictionary<string, Region>();
                    foreach (KeyValuePair<string,Dictionary<string,object>> entry in dct["input_regions"])
                    {
                        region.Add(entry.Key,UtilityFunctions.NodeParser(entry.Value));
                    }

                    inputRegions = regions;
                }

                if (dct.ContainsKey("output_regions"))
                {
                    Dictionary<string,Region> regions = new Dictionary<string, Region>();
                    foreach (KeyValuePair<string,Dictionary<string,object>> entry in dct["output_regions"])
                    {
                        region.Add(entry.Key,UtilityFunctions.NodeParser(entry.Value));
                    }

                    outputRegions = regions;
                }

                if (dct.ContainsKey("recipe"))
                {
                    recipe = MachineRecipe.FromDict((Dictionary<string,object>)dct["recipe"]);
                }

                base.Set(dct);
            }

            /*
            * Cache Methods
            */

            public override void RemoveFromCache()
            {
                if (inputRegions != null)
                {
                    foreach (var region in inputRegions.Values())
                    {
                        region.RemoveFromCache();
                    }
                }

                if (outputRegions != null)
                {
                    foreach (var region in outputRegions.Values())
                    {
                        region.RemoveFromCache();
                    }
                }

                recipe.RemoveFromCache();

                base.RemoveFromCache();
            }

            public override void AddToCache()
            {
                if (inputRegions != null)
                {
                    foreach (var region in inputRegions.Values())
                    {
                        region.AddToCache();
                    }
                }

                if (outputRegions != null)
                {
                    foreach (var region in outputRegions.Values())
                    {
                        region.AddToCache();
                    }
                }

                recipe.AddToCache();

                base.AddToCache();
            }

            /*
            * Update Methods
            */

            public override void LateConstructUpdate()
            {
                if (inputRegions != null)
                {
                    foreach (var region in inputRegions.Values())
                    {
                        region.LateConstructUpdate();
                    }
                }

                if (outputRegions != null)
                {
                    foreach (var region in outputRegions.Values())
                    {
                        region.LateConstructUpdate();
                    }
                }

                recipe.LateConstructUpdate();

                base.LateConstructUpdate();
            }

            public override void DeepUpdate()
            {

                if (inputRegions != null)
                {
                    foreach (var region in inputRegions.Values())
                    {
                        region.DeepUpdate();
                    }
                }

                if (outputRegions != null)
                {
                    foreach (var region in outputRegions.Values())
                    {
                        region.DeepUpdate();
                    }
                }

                recipe.DeepUpdate();

                base.DeepUpdate();

                UpdatedAttribute("input_regions","update");
                UpdatedAttribute("output_regions","update");
                UpdatedAttribute("recipe","update");
            }

            public override void ShallowUpdate()
            {
                base.ShallowUpdate();

                UpdatedAttribute("input_regions","update");
                UpdatedAttribute("output_regions","update");
                UpdatedAttribute("recipe","update");
            }

            /*
            * Utility Methods
            */

            public void ComputeType()
            {
                string type = null;

                if (inputRegions == null && outputRegions == null)
                {
                    type = "useless";
                }
                else if (inputRegions == null && outputRegions != null)
                {
                    if (outputRegions.Count() > 0)
                    {
                        type = "generator";
                    }
                    else
                    {
                        type = "useless";
                    }
                }
                else if (inputRegions != null && outputRegions == null)
                {
                    if (inputRegions.Count() > 0)
                    {
                        type = "consumer";
                    }
                    else    
                    {
                        type = "useless";
                    }
                }
                else
                {
                    type = "transformer";
                }

                machineType = type;
                UpdatedAttribute("machine_type","set");
            }

        }

        public class MachineRecipe : Node
        {

            /*
            * Private Attributes
            */

            private double _processTime;
            private Dictionary<string,int> _inputThingQuantities = null;
            private Dictionary<string,int> _outputThingQuantities = null;
            
            /*
            * Constructors
            */
            public MachineRecipe(double processTime, Dictionary<string,int> inputThingQuantities = null, 
                                 Dictionary<string,int> outputThingQuantities = null, string type = "", string name = "", 
                                 string uuid = null, Node parent = null, bool appendType = true )
            : base(appendType ? "machine-recipe." + type : type, name, uuid, parent, appendType)
            {
                this.processTime = processTime;
                this.inputThingQuantities = (inputThingQuantities != null) ? inputThingQuantities : new Dictionary<string, int>();
                this.outputThingQuantities = (outputThingQuantities != null) ? outputThingQuantities : new Dictionary<string, int>();
            }

            public static new FromDict(Dictionary<string,object> dct)
            {
                return new MachineRecipe(
                    processTime: (double)dct["process_time"],
                    inputThingQuantities: (Dictionary<string,int>)dct["input_thing_quantities"],
                    outputThingQuantities: (Dictionary<string,int>)dct["output_thing_quantities"],
                    type: dct.ContainsKey("type") ? (string)dct["type"] : "",
                    name: dct.ContainsKey("name") ? (string)dct["name"] : "",
                    uuid: dct.ContainsKey("uuid") ? (string)dct["uuid"] : null,
                    appendType: !dct.ContainsKey("type")
                );
            }

            public override Dictionary<string, object> ToDict()
            {
                var msg = base.ToDict();
                msg.Add("process_time", processTime);
                msg.Add("input_thing_quantities",inputThingQuantities);
                msg.Add("output_thing_quantities",outputThingQuantities);
                return msg;
            }

            /*
            * Public Acessors Modifiers
            */

            public double processTime
            {
                get
                {
                    return _processTime;
                }

                set
                {
                    if (_processTime != value)
                    {
                        _processTime = value;
                        UpdatedAttribute("process_time","set");
                    }
                }
            }

            public Dictionary<string,int> inputThingQuantities
            {
                get
                {
                    return _inputThingQuantities;
                }

                set
                {
                    if (_inputThingQuantities != value)
                    {
                        _inputThingQuantities = value;
                        UpdatedAttribute("input_thing_quantities","set");
                    }
                }
            }

            public void AddInputThingQuantity(string thingType, int quantity, bool allowUpdate = false)
            {
                string verb = "add";

                if (inputThingQuantities.ContainsKey(thingType))
                {
                    if (!allowUpdate)
                    {
                        throw new System.Exception("Thing quantity is about to be overrided, however override is not allowed");
                    }
                    else
                    {
                        verb = "update";
                        inputThingQuantities[thingType] = quantity;
                    }
                }
                else
                {
                    inputThingQuantities.Add(thingType,quantity);
                }

                UpdatedAttribute("input_thing_quantities",verb);
            }

            public void DeleteInputThingQuantity(string thingType)
            {
                if (!inputThingQuantities.ContainsKey(thingType))
                {
                    throw new System.Exception("No such thing in input quantities");
                }

                inputThingQuantities.Remove(thingType);
                UpdatedAttribute("input_thing_quantities","delete");
            }

            public int GetOutputThingQuantity(string thingType)
            {
                return inputThingQuantities[thingType];
            }

            public Dictionary<string,int> outputThingQuantities
            {
                get
                {
                    return _outputThingQuantities;
                }

                set
                {
                    if (_outputThingQuantities != value)
                    {
                        _outputThingQuantities = value;
                        UpdatedAttribute("output_thing_quantities","set");
                    }
                }
            }

            public void AddOutputThingQuantitiy(string thingType, int quantity, bool allowUpdate = false)
            {
                string verb = "add";

                if (outputThingQuantities.ContainsKey(thingType))
                {
                    if (!allowUpdate)
                    {
                        throw new System.Exception("Thing quantity is about to be overrided, however override is not allowed");
                    }
                    else
                    {
                        verb = "update";
                        outputThingQuantities[thingType] = quantity;
                    }
                }
                else
                {
                    outputThingQuantities.Add(thingType,quantity);
                }

                UpdatedAttribute("output_thing_quantities",verb);
            }

            public void DeleteOutputThingQuantity(string thingType)
            {
                if (!outputThingQuantities.ContainsKey(thingType))
                {
                    throw new System.Exception("No such thing in output quantities");
                }

                outputThingQuantities.Remove(thingType);
                UpdatedAttribute("output_thing_quantities","delete");
            }

            public int GetOutputThingQuantity(string thingType)
            {
                return outputThingQuantities[thingType];
            }

            public override void Set(Dictionary<string, object> dct)
            {
                if (dct.ContainsKey("process_time"))
                {
                    processTime = (double)dct["process_time"];
                }

                if (dct.ContainsKey("input_thing_quantities"))
                {
                    inputThingQuantities = (Dictionary<string,int>)dct["input_thing_quantities"];
                }

                if (dct.ContainsKey("output_thing_quantities"))
                {
                    outputThingQuantities = (Dictionary<string,int>)dct["output_thing_quantities"];
                }
                
                base.Set(dct);
            }

            /*
            * Update methods
            */

            public override void DeepUpdate()
            {
                base.DeepUpdate();

                UpdatedAttribute("process_time","update");
                UpdatedAttribute("input_thing_quantities","update");
                UpdatedAttribute("output_thing_quantities","update");
            }

            public override void ShallowUpdate()
            {
                base.ShallowUpdate();

                UpdatedAttribute("process_time","update");
                UpdatedAttribute("input_thing_quantities","update");
                UpdatedAttribute("output_thing_quantities","update");
            }

        }
    }
}