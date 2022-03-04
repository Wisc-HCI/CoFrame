import { arrayMove, deleteAction } from './helpers';
import lodash from 'lodash';
import { urdf } from "./robot";
import { FiClipboard, FiBriefcase, FiGrid, FiBox, FiLogOut, FiMoreHorizontal, FiLayers, FiFeather } from "react-icons/fi";
import { DATA_TYPES, TYPES, EXTRA_TYPES, SIMPLE_PROPERTY_TYPES } from 'simple-vp';

import typeInfo from './typeInfo';
// import { performPoseProcess } from './planner-worker';
import { instanceTemplateFromSpec } from 'simple-vp/dist/components';

import * as Comlink from 'comlink';
/* eslint-disable import/no-webpack-loader-syntax */
import Worker from 'worker-loader!./planner-worker';

const cleanedObjectType = (objectType) => lodash.pick(objectType, ['name', 'properties', 'type']);

import {
  LocationIconStyled,
  PrimitiveIconStyled,
  MachineIconStyled,
  ProcessIconStyled,
  SkillIconStyled,
  ThingIconStyled,
  WaypointIconStyled,
  ContainerIconStyled
} from './typeInfo/icons';

export const EvdSlice = (set, get) => ({
  solver: null,
  programSpec: {
    drawers: [
      // Icon is FiGrid, otherwise no icons show in the drawer
      { title: "Machines", dataType: DATA_TYPES.REFERENCE, objectType: 'machineType', icon: MachineIconStyled },
      { title: "Processes", dataType: DATA_TYPES.REFERENCE, objectType: 'processType', icon: ProcessIconStyled },
      { title: "Locations", dataType: DATA_TYPES.REFERENCE, objectType: 'locationType', icon: LocationIconStyled },
      { title: "Waypoints", dataType: DATA_TYPES.REFERENCE, objectType: 'waypointType', icon: WaypointIconStyled },
      { title: "Things", dataType: DATA_TYPES.REFERENCE, objectType: 'thingType', icon: ThingIconStyled },
      { title: "Containers", dataType: DATA_TYPES.INSTANCE, objectTypes: ['trajectoryType', 'hierarchicalType', 'skillType'], icon: ContainerIconStyled },
      { title: "Skills", dataType: DATA_TYPES.CALL, objectType: 'skillType', icon: SkillIconStyled },
      { title: "Actions", dataType: DATA_TYPES.INSTANCE, objectTypes: ['delayType', 'breakpointType', 'gripperType', 'machineInitType', 'processStartType', 'processStopType', 'processWaitType', 'moveTrajectoryType', 'moveUnplannedType'], icon: PrimitiveIconStyled }
    ],
    objectTypes: typeInfo,
  },
  programData: {},
  // All the old stuff below
  data: {
    "program-484de43e-adaa-4801-a23b-bca38e211365": {
      "name": "Knife Assembly",
      "editable": true,
      "deleteable": false,
      "description": "The top-level program",
      "parameters": {},
      "children": [],
      "transform": { "x": 0, "y": 0 }
    }
  },
  // A macro for updating the entire program from raw data
  setData: (data) => set((state) => {

    const newData = lodash.mapValues(data, d => {
      if (d.dataType === DATA_TYPES.INSTANCE) {
        const defaultv = instanceTemplateFromSpec(d.type, state.programSpec.objectTypes[d.type], false);
        console.log({...d,properties:{...defaultv.properties,...d.properties}})
        return {...d,properties:{...defaultv.properties,...d.properties}};
      } else { return d }
    })
    state.programData = newData
  }),
  // setData: (data) => set((_) => ({ programData: data})),
  updatePoseJoints: (id, value, process) => set(state => {
    state.programData[id].joints = value;
    state.processes[id] = process;
  }),
  updatePlanProcess: (newData, process) => set(state => {
    if (newData) {
      state.programData = lodash.merge(state.programData, newData)
    }
    state.processes.planProcess = process
  }),
  performPoseProcess: async (id) => {
    console.log('starting ', id)
    const currentProcess = get().processes[id];
    if (currentProcess) {
      console.log('terminating process for ', id)
      currentProcess.terminate();
    }
    const workerInstance = new Worker();
    get().updatePoseJoints(id, null, workerInstance);
    const workerLib = Comlink.wrap(workerInstance);
    const result = await workerLib.performPoseProcess({ urdf, pose: get().programData[id], scene: {} });
    console.log(result)
    get().updatePoseJoints(id, result, null);
  },
  performPlanProcess: async () => {
    console.log('starting plan processing')
    const currentProcess = get().processes.planProcess;
    if (currentProcess) {
      console.log('terminating current plan process')
      currentProcess.terminate();
    }
    const workerInstance = new Worker();
    get().updatePlanProcess(null, workerInstance);
    const workerLib = Comlink.wrap(workerInstance);
    const result = await workerLib.performStepProcess({ urdf, programData: get().programData, objectTypes: lodash.mapValues(get().programSpec.objectTypes, cleanedObjectType) });
    console.log(result)
    get().updatePlanProcess(result, null);
  },
  processes: {}
});