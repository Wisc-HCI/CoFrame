// import { arrayMove, deleteAction } from './helpers';
import {pick, omit, mapValues} from "lodash";
// import { FiClipboard, FiBriefcase, FiGrid, FiBox, FiLogOut, FiMoreHorizontal, FiLayers, FiFeather } from "react-icons/fi";
import { DATA_TYPES } from "simple-vp";

import typeInfo from "./typeInfo";
// import { performPoseProcess } from './planner-worker';
import { instanceTemplateFromSpec } from "simple-vp";
import useCompiledStore from './CompiledStore';
import * as Comlink from "comlink";
/* eslint-disable import/no-webpack-loader-syntax */
import PlannerWorker from "./planner-worker?worker";

// const plannerWorkerUrl = new URL('./planner-worker.js',import.meta.url);
// const workerInstance = new ComlinkWorker(plannerWorkerUrl,{});
// console.warn('workerInstance',workerInstance)

import {
  LocationIconStyled,
  PrimitiveIconStyled,
  MachineIconStyled,
  ProcessIconStyled,
  SkillIconStyled,
  ThingIconStyled,
  WaypointIconStyled,
  ContainerIconStyled,
  ToolIconStyled,
} from "./typeInfo/icons";

const cleanedObjectType = (objectType) =>
  pick(objectType, ["name", "properties", "type"]);

export const EvdSlice = (set, get) => ({
  solver: null,
  programSpec: {
    drawers: [
      // Icon is FiGrid, otherwise no icons show in the drawer
      {
        title: "Machines",
        dataType: DATA_TYPES.REFERENCE,
        objectType: "machineType",
        icon: MachineIconStyled,
      },
      {
        title: "Processes",
        dataType: DATA_TYPES.REFERENCE,
        objectType: "processType",
        icon: ProcessIconStyled,
      },
      {
        title: "Locations",
        dataType: DATA_TYPES.REFERENCE,
        objectType: "locationType",
        icon: LocationIconStyled,
      },
      {
        title: "Waypoints",
        dataType: DATA_TYPES.REFERENCE,
        objectType: "waypointType",
        icon: WaypointIconStyled,
      },
      {
        title: "Things",
        dataType: DATA_TYPES.REFERENCE,
        objectType: "thingType",
        icon: ThingIconStyled,
      },
      {
        title: "Tools",
        dataType: DATA_TYPES.REFERENCE,
        objectType: "toolType",
        icon: ToolIconStyled,
      },
      {
        title: "Containers",
        dataType: DATA_TYPES.INSTANCE,
        objectTypes: ["trajectoryType", "hierarchicalType", "skillType"],
        icon: ContainerIconStyled,
      },
      {
        title: "Skills",
        dataType: DATA_TYPES.CALL,
        objectType: "skillType",
        icon: SkillIconStyled,
      },
      {
        title: "Actions",
        dataType: DATA_TYPES.INSTANCE,
        objectTypes: [
          "delayType",
          "moveGripperType",
          "machineInitType",
          "processStartType",
          "processWaitType",
          "moveTrajectoryType",
          "breakpointType",
        ],
        icon: PrimitiveIconStyled,
      },
    ],
    objectTypes: typeInfo,
  },
  programData: {},
  // All the old stuff below
  // data: {
  //   "program-484de43e-adaa-4801-a23b-bca38e211365": {
  //     "name": "Knife Assembly",
  //     "editable": true,
  //     "deleteable": false,
  //     "description": "The top-level program",
  //     "parameters": {},
  //     "children": [],
  //     "transform": { "x": 0, "y": 0 }
  //   }
  // },
  // A macro for updating the entire program from raw data
  addAgent: (data) =>
    set((state) => {
      state.programData = { ...state.programData, ...data };
    }),
  replaceAgent: (newData) => set((state)=>{
    const agent = Object.values(newData).filter(d=>d.type==='robotAgentType')[0];
    // Delete any links, meshes, collisionBodies, collisionShapes that are associated with this agent
    Object.values(state.programData).forEach(value=>{
      if (value.type === 'linkType' && value.properties?.agent !== agent.id) {
        if (value.properties.mesh && state.programData[value.properties.mesh]) {
          // Delete mesh
          delete state.programData[value.properties.mesh]
        }
        if (value.properties.collision && state.programData[value.properties.collision]) {
          state.programData[value.properties.collision].properties.componentShapes.forEach(componentShape=>{
            // delete collisionShape
            delete state.programData[componentShape]
          })
          // delete collisionBody
          delete state.programData[value.properties.collision]
        }
        // Delete link
        delete state.programData[value.id]
      }
    });
    
    // Move over robot zones
    Object.values(state.programData)
        .filter(d=>d.type==='zoneType' && state.programData[d.properties.agent].type === 'robotAgentType' && d.properties.agent !== agent.id)
        .forEach((zone)=>{
          state.programData[zone.id].properties.agent = agent.id
    });

    // Delete the agent
    Object.values(state.programData).filter(d=>d.type==='robotAgentType').forEach(robotAgent=>{
      delete state.programData[robotAgent.id]
    })

    state.programData = {...state.programData,...newData}
  }),
  setData: (data) =>
    set((state) => {
      const {tabs, activeTab, ...programData} = data;
      const newData = mapValues(programData, (d) => {
        if (d.dataType === DATA_TYPES.INSTANCE) {
          const defaultv = instanceTemplateFromSpec(
            d.type,
            state.programSpec.objectTypes[d.type],
            false
          );
          return {
            ...d,
            properties: {
              ...defaultv.properties,
              ...omit(d.properties, [
                "status",
                "compiled",
                "compileFn",
                "updateFields",
              ]),
            },
          };
        } else {
          return d;
        }
      });
      // console.log(newData);
      state.programData = newData;
      state.loaded = true;
      if (tabs) {
        state.tabs = tabs
      }
      if (activeTab) {
        state.activeTab = activeTab
      }
    }),
  // setData: (data) => set((_) => ({ programData: data})),
  updatePoseJoints: (id, value, process) =>
    set((state) => {
      state.programData[id].joints = value;
      state.processes[id] = process;
    }),
  updatePlanProcess: (newData, process) =>
    set((state) => {
      if (newData) {
        let reviewableChanges = 0;
        // state.programData = lodash.merge(state.programData, newData);
        Object.keys(newData).forEach((entry) => {
          reviewableChanges += 1;
          newData[entry].properties.pendingChanges = 0;
          Object.keys(newData[entry].properties)
            .forEach((field) => {
              if (field !== 'compiled') {
                state.programData[entry].properties[field] =
                  newData[entry].properties[field];
              }
          });
        });
        state.reviewableChanges += reviewableChanges;
      }
      state.processes.planProcess = process;
      // console.log(useCompiledStore.getState())
    }),
  performCompileProcess: async () => {
    console.log('starting plan processing')
    const currentProcess = get().processes.planProcess;
    if (currentProcess) {
      console.log('terminating current plan process')
      currentProcess.terminate();
    }
    // Create a new worker
    const plannerWorker = new PlannerWorker();
    const { performCompileProcess } = Comlink.wrap(plannerWorker);
    get().updatePlanProcess(null, plannerWorker);
    const programData = get().programData;
    console.log('step 1')
    const result = await performCompileProcess({
      programData,
      compiledData: useCompiledStore.getState(),
      objectTypes: mapValues(
        get().programSpec.objectTypes,
        cleanedObjectType
      ),
    });
    get().updatePlanProcess(result.data, null);

    // // Update the compiled store
    for (const key in result.compiledData) {
      useCompiledStore.setState({[key]:result.compiledData[key]})
    }
  },
  processes: {},
  reviewableChanges: 0,
  // setReviewableChanges: (reviewableChanges) => set({reviewableChanges})
});
