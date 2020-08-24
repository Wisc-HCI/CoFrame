﻿using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Cobots
{
    [System.Serializable]
    public class UtilityFunctions
    {
        public static Node PrimitiveParser(Dictionary<string,object> dct) 
        {
            string[] type = ((string)dct["type"]).Split('.');
            string exactType = type[type.Length - 2];

            Node node = null;
            switch (exactType)
            {
                case "program":
                    node = Program.FromDict(dct);
                    break;

                case "task":
                    node = Task.FromDict(dct);
                    break;
                case "close-gripper":
                    node = CloseGripper.FromDict(dct);
                    break;
                case "open-gripper":
                    node = OpenGripper.FromDict(dct);
                    break;
                case "pick-and-place":
                    node = PickAndPlace.FromDict(dct);
                    break;
                case "initialize":
                    node = Initialize.FromDict(dct);
                    break;
                case "machine-blocking-process":
                    node = MachineBlockingProcess.FromDict(dct);
                    break;
                case "loop":
                    node = Loop.FromDict(dct);
                    break;

                case "primitive":
                    node = Primitive.FromDict(dct);
                    break;
                case "move-trajectory":
                    node = MoveTrajectory.FromDict(dct);
                    break;
                case "move-unplanned":
                    node = MoveUnplanned.FromDict(dct);
                    break;
                case "delay":
                    node = Delay.FromDict(dct);
                    break;
                case "gripper":
                    node = Gripper.FromDict(dct);
                    break;
                case "machine-primitive":
                    node = MachinePrimitive.FromDict(dct);
                    break;
                case "machine-start":
                    node = MachineStart.FromDict(dct);
                    break;
                case "machine-wait":
                    node = MachineWait.FromDict(dct);
                    break;
                case "machine-stop":
                    node = MachineStop.FromDict(dct);
                    break;
                case "machine-initialize":
                    node = MachineInitialize.FromDict(dct);
                    break;
                case "breakpoint":
                    node = Breakpoint.FromDict(dct);
                    break;

                case "reach-sphere":
                    node = ReachSphere.FromDict(dct);
                    break;
                case "pinch-point":
                    node = PinchPoint.FromDict(dct);
                    break;
                case "collision-mesh":
                    node = CollisionMesh.FromDict(dct);
                    break;
                case "occupancy-zone":
                    node = OccupancyZone.FromDict(dct);
                    break;
                case "environment":
                    node = Environment.FromDict(dct);
                    break;

                default:
                    throw new System.Exception("Could not parse primitive supplied.");
            }

            return node;
        }

        public static string GetType(Node n)
        {
            string[] type = n.type.Split('.');
            string exactType = type[type.Length - 2];
            return exactType;
        }
    }
}