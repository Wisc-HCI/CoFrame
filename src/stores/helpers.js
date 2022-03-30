import lodash from 'lodash';
import { GRIPPER_CONFIGURATIONS, GRIPPER_FRAMES, GRIPPER_PARENTS } from './gripper';
import { Quaternion, Vector3, Group, Object3D } from 'three';
import { ConvexGeometry } from 'three-stdlib';
import { EVD_MESH_LOOKUP } from './initialSim';
import { DATA_TYPES } from 'simple-vp';
import { REFERENCEABLE_OBJECTS } from './Constants';

Object3D.DefaultUp.set(0, 0, 1);

export const distance = (pos1, pos2) => {
    return Math.sqrt(Math.pow(pos1.x - pos2.x, 2) + Math.pow(pos1.y - pos2.y, 2) + Math.pow(pos1.z - pos2.z, 2))
}

export const quaternionLog = (quaternion) => {
    let outVec = new Vector3(quaternion.x, quaternion.y, quaternion.z);
    if (Math.abs(quaternion.w) < 1.0) {
        let a = Math.acos(quaternion.w);
        let sina = Math.sin(a);
        if (Math.abs(sina) >= 0.005) {
            let c = a / sina;
            outVec.multiplyScalar(c);
        }
    }
    return [outVec.x, outVec.y, outVec.z]
}

const ROBOT_FRAMES = [
    'base_link',
    'shoulder_link',
    'upper_arm_link',
    'forearm_link',
    'wrist_1_link',
    'wrist_2_link',
    'wrist_3_link',
    'flange',
    'tool0',
    'tool0_endpoint'
]

export const PRIMITIVE_TYPES = [
    'delay', 'gripper', 'machine-initialize', 'process-start', 'process-stop',
    'process-wait', 'move-trajectory', 'move-unplanned', 'breakpoint', 'skill-call'
]

export const HUMAN_ZONE = { radius: 2, height: 3, position: { x: 0, y: -1, z: 1 } }

export const DEFAULT_LOCATION_COLOR = { r: 62, g: 16, b: 102, a: 1 };

export const DEFAULT_WAYPOINT_COLOR = { r: 100, g: 18, b: 128, a: 1 };

export const DEFAULT_TRAJECTORY_COLOR = { r: 209, g: 0, b: 146, a: 1 };

export const UNREACHABLE_COLOR = { r: 204, g: 75, b: 10, a: 1 };

export const OCCUPANCY_ERROR_COLOR = { r: 233, g: 53, b: 152, a: 1 };

export const PINCH_POINT_FIELDS = {
    base_link_inertia___forearm_link: { parent: 'base_link', frame1: 'Base', frame2: 'Forearm', scale: { x: 0, y: 0, z: 0 }, position: { x: 0, y: 0, z: 0 }, color: { r: 0, g: 0, b: 0, a: 0 } },
    base_link_inertia___gripper: { parent: 'base_link', frame1: 'Base', frame2: 'Gripper', scale: { x: 0, y: 0, z: 0 }, position: { x: 0, y: 0, z: 0 }, color: { r: 0, g: 0, b: 0, a: 0 } },
    base_link_inertia___upper_arm_link: { parent: 'base_link', frame1: 'Base', frame2: 'Upper Arm', scale: { x: 0, y: 0, z: 0 }, position: { x: 0, y: 0, z: 0 }, color: { r: 0, g: 0, b: 0, a: 0 } },
    base_link_inertia___wrist_1_link: { parent: 'base_link', frame1: 'Base', frame2: 'Wrist 1', scale: { x: 0, y: 0, z: 0 }, position: { x: 0, y: 0, z: 0 }, color: { r: 0, g: 0, b: 0, a: 0 } },
    base_link_inertia___wrist_2_link: { parent: 'base_link', frame1: 'Base', frame2: 'Wrist 2', scale: { x: 0, y: 0, z: 0 }, position: { x: 0, y: 0, z: 0 }, color: { r: 0, g: 0, b: 0, a: 0 } },
    base_link_inertia___wrist_3_link: { parent: 'base_link', frame1: 'Base', frame2: 'Wrist 3', scale: { x: 0, y: 0, z: 0 }, position: { x: 0, y: 0, z: 0 }, color: { r: 0, g: 0, b: 0, a: 0 } },

    shoulder_link___forearm_link: { parent: 'shoulder_link', frame1: 'Shoulder', frame2: 'Forearm', scale: { x: 0, y: 0, z: 0 }, position: { x: 0, y: 0, z: 0 }, color: { r: 0, g: 0, b: 0, a: 0 } },
    shoulder_link___gripper: { parent: 'shoulder_link', frame1: 'Shoulder', frame2: 'Gripper', scale: { x: 0, y: 0, z: 0 }, position: { x: 0, y: 0, z: 0 }, color: { r: 0, g: 0, b: 0, a: 0 } },
    shoulder_link___wrist_1_link: { parent: 'shoulder_link', frame1: 'Shoulder', frame2: 'Wrist 1', scale: { x: 0, y: 0, z: 0 }, position: { x: 0, y: 0, z: 0 }, color: { r: 0, g: 0, b: 0, a: 0 } },
    shoulder_link___wrist_2_link: { parent: 'shoulder_link', frame1: 'Shoulder', frame2: 'Wrist 2', scale: { x: 0, y: 0, z: 0 }, position: { x: 0, y: 0, z: 0 }, color: { r: 0, g: 0, b: 0, a: 0 } },
    shoulder_link___wrist_3_link: { parent: 'shoulder_link', frame1: 'Shoulder', frame2: 'Wrist 3', scale: { x: 0, y: 0, z: 0 }, position: { x: 0, y: 0, z: 0 }, color: { r: 0, g: 0, b: 0, a: 0 } },

    upper_arm_link___gripper: { parent: 'upper_arm_link', frame1: 'Upper Arm', frame2: 'Gripper', scale: { x: 0, y: 0, z: 0 }, position: { x: 0, y: 0, z: 0 }, color: { r: 0, g: 0, b: 0, a: 0 } },
    upper_arm_link___wrist_1_link: { parent: 'upper_arm_link', frame1: 'Upper Arm', frame2: 'Wrist 1', scale: { x: 0, y: 0, z: 0 }, position: { x: 0, y: 0, z: 0 }, color: { r: 0, g: 0, b: 0, a: 0 } },
    upper_arm_link___wrist_2_link: { parent: 'upper_arm_link', frame1: 'Upper Arm', frame2: 'Wrist 2', scale: { x: 0, y: 0, z: 0 }, position: { x: 0, y: 0, z: 0 }, color: { r: 0, g: 0, b: 0, a: 0 } },
    upper_arm_link___wrist_3_link: { parent: 'upper_arm_link', frame1: 'Upper Arm', frame2: 'Wrist 3', scale: { x: 0, y: 0, z: 0 }, position: { x: 0, y: 0, z: 0 }, color: { r: 0, g: 0, b: 0, a: 0 } },

    forearm_link___gripper: { parent: 'forearm_link', frame1: 'Forearm', frame2: 'Gripper', scale: { x: 0, y: 0, z: 0 }, position: { x: 0, y: 0, z: 0 }, color: { r: 0, g: 0, b: 0, a: 0 } },
    forearm_link___wrist_2_link: { parent: 'forearm_link', frame1: 'Forearm', frame2: 'Wrist 2', scale: { x: 0, y: 0, z: 0 }, position: { x: 0, y: 0, z: 0 }, color: { r: 0, g: 0, b: 0, a: 0 } },
    forearm_link___wrist_3_link: { parent: 'forearm_link', frame1: 'Forearm', frame2: 'Wrist 3', scale: { x: 0, y: 0, z: 0 }, position: { x: 0, y: 0, z: 0 }, color: { r: 0, g: 0, b: 0, a: 0 } },

    // wrist_1_link___gripper: {parent: 'wrist_1_link', frame1: 'Wrist 1', frame2: 'Gripper', scale: {x:0,y:0,z:0}, position: {x:0,y:0,z:0}, color: {r:0,g:0,b:0,a:0}},
    wrist_1_link___wrist_3_link: { parent: 'wrist_1_link', frame1: 'Wrist 1', frame2: 'Wrist 3', scale: { x: 0, y: 0, z: 0 }, position: { x: 0, y: 0, z: 0 }, color: { r: 0, g: 0, b: 0, a: 0 } },

    // wrist_2_link___gripper: {parent: 'wrist_2_link', frame1: 'Wrist 2', frame2: 'Gripper', scale: {x:0,y:0,z:0}, position: {x:0,y:0,z:0}, color: {r:0,g:0,b:0,a:0}},
}

export const queryWorldPose = (model, ref) => {
    const referenceFeature = model[ref];
    console.log('REF FEATURE', { ref, referenceFeature });
    const matrixWorld = referenceFeature.matrixWorld;
    let position = referenceFeature.getWorldPosition(new Vector3());
    let rotation = referenceFeature.getWorldQuaternion(new Quaternion());
    return {
        position: {
            x: position.x,
            y: position.y,
            z: position.z
        },
        rotation: {
            x: rotation.x,
            y: rotation.y,
            z: rotation.z,
            w: rotation.w
        }
    }
}

export const queryLocalPose = (model, ref, localTransform) => {
    const referenceFeature = model[ref];
    // create a temporary group to add to that reference;
    const worldPose = new Group();
    worldPose.position.set(localTransform.position.x, localTransform.position.y, localTransform.position.z);
    worldPose.quaternion.set(localTransform.rotation.x, localTransform.rotation.y, localTransform.rotation.z, localTransform.rotation.w)
    referenceFeature.attach(worldPose);
    const local = {
        position: {
            x: worldPose.position.x,
            y: worldPose.position.y,
            z: worldPose.position.z
        },
        rotation: {
            x: worldPose.quaternion.x,
            y: worldPose.quaternion.y,
            z: worldPose.quaternion.z,
            w: worldPose.quaternion.w
        }
    }
    worldPose.removeFromParent();
    return local
}

export const createStaticEnvironment = (model) => {
    return Object.values(model).filter(item => item.userData.parent !== 'world' && item.userData.collisionInfo).map(item => {
        // TODO: Create static collision info here
    })
}

export const createEnvironmentModel = (programData) => {
    let added = true;
    let model = {};
    model.world = new Group();
    while (added) {
        added = false
        Object.values(programData).filter(i => !Object.keys(model).includes(i.id) && i.dataType === DATA_TYPES.INSTANCE && REFERENCEABLE_OBJECTS.includes(i.type)).forEach(item => {
            const parentId = item.properties.relativeTo ? item.properties.relativeTo : 'world';
            if (model[parentId]) {
                model[item.id] = new Group();
                model[item.id].userData.parent = parentId;
                // TODO: rework collisions to have intelligble data here
                if (false) {
                    model[item.id].userData.collisionInfo = true;
                }
                model[item.id].position.set(item.properties.position.x, item.properties.position.y, item.properties.position.z);
                model[item.id].quaternion.set(item.properties.rotation.x, item.properties.rotation.y, item.properties.rotation.z, item.properties.rotation.w);
                model[parentId].add(model[item.id]);
                added = true;
            }
        })
    }
    return model
}

export const likFramesToTransforms = (frames, model, frame) => {
    const relativeFrameId = frame ? frame : 'world';
    console.log({frames,model,frame})
    const linkTransforms = lodash.mapValues(frames, (frameData) => {
        const poseWorld = {
            position: {
                x: frameData.translation[0],
                y: frameData.translation[1],
                z: frameData.translation[2]
            },
            rotation: {
                w: frameData.rotation[3],
                x: frameData.rotation[0],
                y: frameData.rotation[1],
                z: frameData.rotation[2]
            }
        }
        const poseLocal = queryLocalPose(model, relativeFrameId, poseWorld)
        return {
            frame,
            ...poseLocal
        }
    })
    return linkTransforms
}

export const likStateToData = (state, model, frame) => {
    console.log('PRE-PARSED STATE DATA',state);
    console.log({frames:state.frames,model,frame})
    const data = {
        joints: state.joints,
        links: likFramesToTransforms(state.frames, model, frame),
        proximity: state.proximity,
    }
    console.log('PARSED STATE DATA',data)
    return data
}

export const typeToKey = (type) => {
    let key;
    switch (type) {
        case 'trajectory':
            key = 'trajectories';
            break;
        case 'collisionMesh':
            key = 'collisionMeshes';
            break;
        default:
            key = type + 's'
    }
    return key;
}

function* range(start, end, step = 1) {
    for (let i = start; i <= end; i += step) {
        yield i;
    }
}

export function objectMap(object, mapFn) {
    return Object.keys(object).reduce(function (result, key) {
        result[key] = mapFn(object[key], key)
        return result
    }, {})
}

export function arrayMove(arr, old_index, new_index) {
    while (old_index < 0) {
        old_index += arr.length;
    }
    while (new_index < 0) {
        new_index += arr.length;
    }
    if (new_index >= arr.length) {
        var k = new_index - arr.length + 1;
        while (k--) {
            arr.push(undefined);
        }
    }
    arr.splice(new_index, 0, arr.splice(old_index, 1)[0]);
    return arr; // for testing purposes
};

export function itemTransformMethod(state, id) {
    let idIncluded = false;
    let transformMethod = 'inactive';

    // This variable determines whether the encountered translate/rotate applies to the item being searched
    let isTransformActive = false;

    state.focus.some(f => {
        let focusItemType = state.programData[f]?.type
        let focusTypeInfo = focusItemType ? Object.keys(state.programSpec.objectTypes[focusItemType].properties) : null;
        // Reset the isTransformActive, as the next translate/rotate would reference this new object
        if (focusTypeInfo && idIncluded && focusTypeInfo.includes("position")) {
            isTransformActive = false;
        }

        // Item exists in the focus array
        if (f === id) {
            idIncluded = true;
            isTransformActive = true;
        }

        // If item exists, and still has the potential focus of the translate/rotate, determine if translation/rotation applies
        if (idIncluded && isTransformActive && f === 'translate') {
            transformMethod = 'translate';
            return true;
        }
        if (idIncluded && isTransformActive && f === 'rotate') {
            transformMethod = 'rotate';
            return true;
        }
        return false;
    });

    return transformMethod;
}

export function deleteAction(data, uuid) {
    if (data[uuid].children) {
        /* Delete as a hierarchical */
        let children = data[uuid].children;
        let updated = lodash.omit(data, uuid);
        children.forEach(child => {
            updated = deleteAction(data, child)
        })
        return updated;
    } else {
        return lodash.omit(data, uuid)
    }
};

export const occupancyOverlap = (position, occupancyZones) => {
    let overlap = false
    let zones = Object.values(occupancyZones).filter(v => v.type === 'zoneType');
    for (let i = 0; i < zones.length; i++) {//.forEach(zone => {
        let zone = zones[i];
        const xOverlap = position.x < zone.properties.position.x + zone.properties.scale.x / 2 && position.x > zone.properties.position.x - zone.properties.scale.x / 2;
        const yOverlap = position.y < zone.properties.position.z + zone.properties.scale.z / 2 && position.y > zone.properties.position.z - zone.properties.scale.z / 2;
        if (xOverlap && yOverlap) {
            overlap = true
        }
    }
    return overlap
}

export function flattenProgram(primitives, skills, parentData) {

    let flattenedPrimitives = [];
    let flattenedSkills = [];

    primitives.forEach(primitive => {
        if (primitive.type.includes('hierarchical')) {
            let newPrimitive = lodash.omit(primitive, 'primitives');
            newPrimitive.children = primitive.primitives.map(primitive => primitive.uuid);
            newPrimitive.parentData = { type: 'primitive', uuid: primitive }
            flattenedPrimitives.push(newPrimitive);
            const primitiveChildren = flattenProgram(primitive.primitives, [], { type: 'primitive', uuid: primitive.uuid })[0];
            flattenedPrimitives = [...flattenedPrimitives, ...primitiveChildren];
        } else {
            flattenedPrimitives.push({ ...primitive, parentData })
        }
    });
    skills.forEach(skill => {
        if (skill.type.includes('hierarchical')) {
            let newSkill = lodash.omit(skill, 'primitives');
            newSkill.children = skill.primitives.map(primitive => primitive.uuid);
            flattenedSkills.push(newSkill);
            const primitiveChildren = flattenProgram(skill.primitives, [], { type: 'skill', uuid: skill.uuid })[0];
            flattenedPrimitives = [...flattenedPrimitives, ...primitiveChildren]
        }
    })

    return [flattenedPrimitives, flattenedSkills]
}

function executableTrajectory(trajectory, context) {
    let executable = { uuid: trajectory.uuid, sequence: [], trace: null }
    if (!trajectory.start_location_uuid || !trajectory.end_location_uuid || !trajectory.trace) {
        console.log('no start or end')
        return null
    }
    executable.sequence.push(context[trajectory.start_location_uuid])
    trajectory.waypoint_uuids.forEach(waypoint_uuid => {
        executable.sequence.push(context[waypoint_uuid])
    })
    executable.sequence.push(context[trajectory.end_location_uuid]);
    executable.trace = trajectory.trace;
    executable.vertices = traceToVertices(trajectory.trace);
    executable.volume = verticesToVolume(executable.vertices);
    executable.eePoseScores = traceToEEPoseScores(trajectory.trace);
    // console.log(executable)
    return executable
}

export function executableMachine(machine, context) {
    let executable = { ...machine, inputData: [], outputData: [] };
    Object.keys(machine.inputs).forEach(thingType => {
        const thingInfo = context[thingType];
        machine.inputs[thingType].forEach(outputData => {
            const region = context[outputData.region_uuid];
            // For now, assume only one is created
            executable.inputData.push({ thing: thingInfo, region })
        })
    })
    Object.keys(machine.outputs).forEach(thingType => {
        const thingInfo = context[thingType];
        machine.outputs[thingType].forEach(outputData => {
            const region = context[outputData.region_uuid];
            // For now, assume only one is created
            executable.outputData.push({ thing: thingInfo, region, placeholder: context[outputData.placeholder_uuids[0]] })
        })
    })

    return executable
}

function executablePrimitiveInner(primitiveId, state, context) {
    let full = {};
    let executable = [];
    const primitive = state.data[primitiveId];
    if (!primitive) {
        return null
    }
    if (primitive.children) {
        if (primitive.children.some(childId => {
            const inner = executablePrimitiveInner(childId, state, context);
            if (inner === null || inner.children.includes(null)) {
                return true
            } else {
                executable = [...executable, ...inner.children]
                full = { ...full, ...inner.all, [childId]: inner.children }
                return false
            }
        })) {
            // There was a null response
            return null
        } else {
            return { children: executable, all: { ...full, [primitive.uuid]: executable } }
        }
    } else if (primitive.type.includes('skill-call')) {
        let innerContext = { ...context };
        if (Object.keys(primitive.parameters).some(parameterKey => {
            const value = primitive.parameters[parameterKey];
            if (parameterKey !== 'skill_uuid' && context[value]) {
                innerContext[parameterKey] = context[value]
                return false
            } else if (parameterKey !== 'skill_uuid') {
                return true
            }
            return false
        })) {
            // There was a null param
            return null
        }
        const calledSkill = state.data.skills[primitive.parameters.skill_uuid];
        if (!calledSkill) {
            return null
        }
        if (calledSkill.children.some(childId => {
            const inner = executablePrimitiveInner(childId, state, innerContext);
            if (inner === null || inner.children.includes(null)) {
                return true
            } else {
                executable = [...executable, ...inner.children]
                return false
            }
        })) {
            // There was a null response
            return null
        } else {
            return { children: executable, all: { [primitive.uuid]: executable } }
        }
    } else if (primitive.type === 'node.primitive.breakpoint.' || primitive.type === 'node.primitive.delay.') {
        return { children: [primitive], all: { [primitive.uuid]: primitive } }
    } else if (primitive.type === 'node.primitive.gripper.') {
        if (context[primitive.parameters.thing_uuid]) {
            const expanded = { ...primitive, parameters: { ...primitive.parameters, thing_uuid: context[primitive.parameters.thing_uuid] } }
            return { children: [expanded], all: { [primitive.uuid]: expanded } }
        } else {
            return null
        }
    } else if (primitive.type.includes('machine-primitive')) {
        if (context[primitive.parameters.machine_uuid]) {
            const expanded = { ...primitive, parameters: { ...primitive.parameters, machine_uuid: context[primitive.parameters.machine_uuid] } }
            return { children: [expanded], all: { [primitive.uuid]: expanded } }
        } else {
            return null
        }
    } else if (primitive.type === 'node.primitive.move-trajectory.') {
        if (primitive.parameters.trajectory_uuid && context[primitive.parameters.trajectory_uuid]) {
            const expanded = { ...primitive, parameters: { ...primitive.parameters, trajectory_uuid: context[primitive.parameters.trajectory_uuid] } }
            return { children: [expanded], all: { [primitive.uuid]: expanded } }
        } else {
            return null
        }
    } else if (primitive.type === 'node.primitive.move-unplanned.') {
        if (context[primitive.parameters.location_uuid]) {
            const expanded = { ...primitive, parameters: { ...primitive.parameters, location_uuid: context[primitive.parameters.location_uuid] } }
            return { children: [expanded], all: { [primitive.uuid]: expanded } }
        } else {
            return null
        }
    }
    return null
}

export function executablePrimitives(state) {
    let context = state.data;
    Object.values(state.data).filter(v => v.type === 'trajectory').forEach(trajectory => {
        context[trajectory.uuid] = executableTrajectory(trajectory, context)
    })
    Object.values(state.data).filter(v => v.type === 'machine').forEach(machine => {
        context[machine.uuid] = executableMachine(machine, context)
    })
    // this should return all the primitives that are run through with the actual program
    const inner = executablePrimitiveInner(state.uuid, state, context);
    let executables = {}
    if (inner) {
        executables = { ...inner.all };
    }

    // this should catch any primitives not called directly through the program that are valid
    Object.values(state.data).filter(v => PRIMITIVE_TYPES.includes(v.type)).forEach(primitive => {
        if (!executables[primitive.uuid]) {
            executables = { ...executables, ...executablePrimitiveInner(primitive.uuid, state, context) }
        }
    })
    return executables

}

const findLastSatisfiedFromReference = (x, fn) => {
    let lastIdx = 0;
    for (let i = 0; i < x.length; i++) {
        if (fn(x[i])) {
            lastIdx = i
        } else {
            break
        }
    }
    return lastIdx
}

const gripperFramesFromValue = (distance) => {
    const idx = findLastSatisfiedFromReference(GRIPPER_CONFIGURATIONS.cmd, v => v > distance);
    console.log(idx)
    let frames = {};
    GRIPPER_FRAMES.forEach(frameName => {
        const rawFrame = GRIPPER_CONFIGURATIONS.capture[frameName][idx];
        frames[frameName] = {
            frame: GRIPPER_PARENTS[frameName],
            translation: {
                x: rawFrame[0][0],
                y: rawFrame[0][1],
                z: rawFrame[0][2]
            },
            rotation: {
                x: rawFrame[1][0],
                y: rawFrame[1][1],
                z: rawFrame[1][2],
                w: rawFrame[1][3]
            }
        }
    })
    return frames
}

export const robotFramesFromPose = (pose) => {
    let frames = {};
    console.log(pose.frames)
    ROBOT_FRAMES.forEach(frame => {
        const rawFrame = pose.frames[frame];
        frames['simulated_' + frame] = {
            translation: {
                x: rawFrame[0][0],
                y: rawFrame[0][1],
                z: rawFrame[0][2]
            },
            rotation: {
                x: rawFrame[1][0],
                y: rawFrame[1][1],
                z: rawFrame[1][2],
                w: rawFrame[1][3]
            }
        }
    })
    return frames
}

const frameFromRegion = (region) => {
    return {
        translation: {
            x: region.center_position.x,
            y: region.center_position.y,
            z: region.center_position.z
        },
        rotation: {
            x: region.center_orientation.x,
            y: region.center_orientation.y,
            z: region.center_orientation.z,
            w: region.center_orientation.w,
        }
    }
}

const robotFramesFromIdx = (idx, trace) => {
    let frames = {};
    ROBOT_FRAMES.forEach(frame => {
        const rawFrame = trace.frames[frame][idx];
        frames['simulated_' + frame] = {
            translation: {
                x: rawFrame[0][0],
                y: rawFrame[0][1],
                z: rawFrame[0][2]
            },
            rotation: {
                x: rawFrame[1][0],
                y: rawFrame[1][1],
                z: rawFrame[1][2],
                w: rawFrame[1][3]
            }
        }
    })
    return frames
}

const pinchColorFromMagnitude = (magnitude = 0) => {
    return { r: 204 + 29 * magnitude, g: 121 - 68 * magnitude, b: 167 - 15 * magnitude, a: 0.3 };
}

const pinchPointVisualsByIdx = (idx, trace) => {
    let pinchPoints = {};
    console.log(trace.pinch_points)
    Object.keys(trace.pinch_points).forEach(pinchPointPair => {
        if (trace.pinch_points[pinchPointPair][idx]) {
            let errorMagnitude = 1 / Math.pow(Math.E, trace.pinch_points[pinchPointPair][idx].gap);
            let errorPosition = trace.pinch_points[pinchPointPair][idx].position;
            console.log({ errorMagnitude, errorPosition })
            pinchPoints[pinchPointPair] = {
                scale: { x: errorMagnitude * 0.1, y: errorMagnitude * 0.1, z: errorMagnitude * 0.1 },
                color: pinchColorFromMagnitude(errorMagnitude),
                position: { x: errorPosition[0], y: errorPosition[1], z: errorPosition[2] }
            }
        } else {
            pinchPoints[pinchPointPair] = {
                scale: { x: 0, y: 0, z: 0 },
                color: { r: 0, g: 0, b: 0, a: 0 },
                position: { x: 0, y: 0, z: 0 }
            }
        }
    })
    return pinchPoints
}

const poseDiff = (pose1, pose2) => {
    console.log(pose1, pose2)
    const translationDistance = Math.sqrt(
        Math.pow(pose1.position.x - pose2.position.x, 2) +
        Math.pow(pose1.position.y - pose2.position.y, 2) +
        Math.pow(pose1.position.z - pose2.position.z, 2)
    )
    console.log(translationDistance)
    const quat1 = Quaternion(pose1.quaternion.x, pose1.quaternion.y, pose1.quaternion.z, pose1.quaternion.w)
    const quat2 = Quaternion(pose2.quaternion.x, pose2.quaternion.y, pose2.quaternion.z, pose2.quaternion.w)
    return { distance: translationDistance, angle: quat1.angleTo(quat2) }
}

const stepsToAnimatedTfs = (steps) => {
    if (steps.length === 0) {
        return {}
    }
    let tempAnimatedTfs = objectMap(steps[0].tfs, _ => ({
        translation: { x: [], y: [], z: [] },
        rotation: { w: [], x: [], y: [], z: [] }
    })
    )
    const tfNames = Object.keys(tempAnimatedTfs);
    console.log(tfNames)
    let timesteps = steps.map(step => step.time)
    steps.forEach(step => {
        tfNames.forEach(tfName => {
            tempAnimatedTfs[tfName].translation.x.push(step.tfs[tfName].translation.x);
            tempAnimatedTfs[tfName].translation.y.push(step.tfs[tfName].translation.y);
            tempAnimatedTfs[tfName].translation.z.push(step.tfs[tfName].translation.z);
            tempAnimatedTfs[tfName].rotation.w.push(step.tfs[tfName].rotation.w);
            tempAnimatedTfs[tfName].rotation.x.push(step.tfs[tfName].rotation.x);
            tempAnimatedTfs[tfName].rotation.y.push(step.tfs[tfName].rotation.y);
            tempAnimatedTfs[tfName].rotation.z.push(step.tfs[tfName].rotation.z);
        })
    })
    // console.log(tempAnimatedTfs)
    const animatedTfs = objectMap(tempAnimatedTfs, (tf, key) => ({
        frame: GRIPPER_PARENTS[key],
        translation: {
            x: interpolateScalar(timesteps, tf.translation.x),
            y: interpolateScalar(timesteps, tf.translation.y),
            z: interpolateScalar(timesteps, tf.translation.z)
        },
        rotation: {
            w: interpolateScalar(timesteps, tf.rotation.w),
            x: interpolateScalar(timesteps, tf.rotation.x),
            y: interpolateScalar(timesteps, tf.rotation.y),
            z: interpolateScalar(timesteps, tf.rotation.z)
        }
    }))
    // for (let timestep of timesteps) {
    //     console.log(animatedTfs['simulated_tool0'].translation.x(timestep))
    // }

    return animatedTfs
}

const stepsToAnimatedPinchPoints = (steps) => {
    if (steps.length === 0) {
        return {}
    }
    let tempAnimatedPinchPoints = objectMap(steps[0].pinchPoints, _ => ({
        position: { x: [], y: [], z: [] },
        scale: { x: [], y: [], z: [] },
        color: { r: [], g: [], b: [] }
    })
    )
    const pinchPointPairs = Object.keys(tempAnimatedPinchPoints);
    // console.log(pinchPointPairs)
    let timesteps = steps.map(step => step.time)
    steps.forEach(step => {
        pinchPointPairs.forEach(pairName => {
            tempAnimatedPinchPoints[pairName].position.x.push(step.pinchPoints[pairName].position.x);
            tempAnimatedPinchPoints[pairName].position.y.push(step.pinchPoints[pairName].position.y);
            tempAnimatedPinchPoints[pairName].position.z.push(step.pinchPoints[pairName].position.z);
            tempAnimatedPinchPoints[pairName].scale.x.push(step.pinchPoints[pairName].scale.x);
            tempAnimatedPinchPoints[pairName].scale.y.push(step.pinchPoints[pairName].scale.y);
            tempAnimatedPinchPoints[pairName].scale.z.push(step.pinchPoints[pairName].scale.z);
            tempAnimatedPinchPoints[pairName].color.r.push(step.pinchPoints[pairName].color.r);
            tempAnimatedPinchPoints[pairName].color.g.push(step.pinchPoints[pairName].color.g);
            tempAnimatedPinchPoints[pairName].color.b.push(step.pinchPoints[pairName].color.b);
        })
    })
    console.log(tempAnimatedPinchPoints)
    const animatedPinchPoints = objectMap(tempAnimatedPinchPoints, pinchPoint => ({
        frame: 'world',
        position: {
            x: interpolateScalar(timesteps, pinchPoint.position.x),
            y: interpolateScalar(timesteps, pinchPoint.position.y),
            z: interpolateScalar(timesteps, pinchPoint.position.z)
        },
        scale: {
            x: interpolateScalar(timesteps, pinchPoint.scale.x),
            y: interpolateScalar(timesteps, pinchPoint.scale.y),
            z: interpolateScalar(timesteps, pinchPoint.scale.z)
        },
        color: {
            r: interpolateScalar(timesteps, pinchPoint.color.r),
            g: interpolateScalar(timesteps, pinchPoint.color.g),
            b: interpolateScalar(timesteps, pinchPoint.color.b),
            a: 0.3
        }
    }))
    console.log(animatedPinchPoints)
    return animatedPinchPoints
}

export function pinchpointAnimationFromExecutable(executable) {
    let steps = [{ time: 0, pinchPoints: PINCH_POINT_FIELDS }]
    let machineProcessing = {};
    let cancelled = false;
    let currentTime = 0;
    let gripperState = 55;
    executable.forEach(chunk => {
        let prevStep = lodash.cloneDeep(steps[steps.length - 1]);
        let duration = 0;
        if (!cancelled) {
            if (chunk.type === 'node.primitive.gripper.') {
                const delta = chunk.parameters.position - gripperState;
                duration = 1000 * Math.abs(delta) / chunk.parameters.speed;

                for (let timeOffset of range(0, duration, 100)) {
                    prevStep.time = currentTime + timeOffset;
                    steps.push(prevStep)
                    prevStep = lodash.cloneDeep(steps[steps.length - 1]);
                }
            } else if (chunk.type === 'node.primitive.delay.') {
                duration = chunk.parameters.duration * 1000;
                steps.push({ ...prevStep, time: currentTime + duration })
            } else if (chunk.type === 'node.primitive.breakpoint.') {
                cancelled = true;
            } else if (chunk.type === 'node.primitive.machine-primitive.machine-initialize.') {
                // Ignore
            } else if (chunk.type === 'node.primitive.machine-primitive.machine-start.') {
                machineProcessing[chunk.parameters.machine_uuid.uuid] = chunk.parameters.machine_uuid.process_time;
            } else if (chunk.type === 'node.primitive.machine-primitive.machine-stop.') {
                // Ignore
            } else if (chunk.type === 'node.primitive.machine-primitive.machine-wait.') {
                if (machineProcessing[chunk.parameters.machine_uuid.uuid]) {
                    duration = machineProcessing[chunk.parameters.machine_uuid.uuid]
                }
            } else if (chunk.type === 'node.primitive.move-trajectory.') {
                duration = chunk.parameters.trajectory_uuid.trace.duration * 1000;
                if (chunk)
                    for (let i = 0; i < chunk.parameters.trajectory_uuid.trace.time_data.length - 1; i++) {
                        prevStep.pinchPoints = { ...prevStep.pinchPoints, ...pinchPointVisualsByIdx(i, chunk.parameters.trajectory_uuid.trace) };
                        prevStep.time = currentTime + chunk.parameters.trajectory_uuid.trace.time_data[i] * 1000;
                        steps.push(prevStep)
                        prevStep = lodash.cloneDeep(steps[steps.length - 1]);
                    }
                if (chunk.parameters.trajectory_uuid.trace.in_timeout) {
                    cancelled = true
                }
            } else if (chunk.type === 'node.primitive.move-unplanned.') {
                duration = 100;
                prevStep.time = prevStep.time + 100;
                steps.push(prevStep)
                prevStep = lodash.cloneDeep(steps[steps.length - 1]);
            }
        }
        currentTime += duration;
        machineProcessing = objectMap(machineProcessing, val => {
            if (val - duration < 0) {
                return 0
            } else {
                return val - duration
            };
        })
    })
    steps.push({ ...lodash.cloneDeep(steps[steps.length - 1]), time: currentTime + 1000 })
    return stepsToAnimatedPinchPoints(steps)
}

export function tfAnimationFromExecutable(executable, startingTfs) {
    let steps = [{ time: 0, tfs: startingTfs }]
    let machineProcessing = {};
    let cancelled = false;
    let currentTime = 0;
    let gripperState = 55;
    let activePlaceholders = [];
    let carriedPlaceholder = null;
    executable.forEach(chunk => {
        let prevTfs = lodash.cloneDeep(steps[steps.length - 1]);
        let duration = 0;
        if (!cancelled) {
            if (chunk.type === 'node.primitive.gripper.') {
                const delta = chunk.parameters.position - gripperState;
                const direction = delta >= 0 ? 1 : -1
                duration = 1000 * Math.abs(delta) / chunk.parameters.speed;

                for (let timeOffset of range(0, duration, 100)) {
                    const tempGripperState = gripperState + chunk.parameters.speed * timeOffset / 1000 * direction;
                    prevTfs.tfs = { ...prevTfs.tfs, ...gripperFramesFromValue(tempGripperState) };
                    prevTfs.time = currentTime + timeOffset;
                    steps.push(prevTfs)
                    prevTfs = lodash.cloneDeep(steps[steps.length - 1]);
                }
                if (direction === -1) {
                    activePlaceholders.forEach(placeholder => {
                        //const {distance, angle} = poseDiff(prevTfs[placeholder],prevTfs['simulated_tool0']);
                        const distance = 0;
                        const angle = 0.2;
                        if (distance < 0.01 && angle < 0.35) {
                            carriedPlaceholder = placeholder
                        }
                    })
                }
                if (direction === 1 && carriedPlaceholder) {
                    carriedPlaceholder = null
                }
            } else if (chunk.type === 'node.primitive.delay.') {
                duration = chunk.parameters.duration * 1000;
                steps.push({ ...prevTfs, time: currentTime + duration })
            } else if (chunk.type === 'node.primitive.breakpoint.') {
                cancelled = true;
            } else if (chunk.type === 'node.primitive.machine-primitive.machine-initialize.') {
                // Ignore
            } else if (chunk.type === 'node.primitive.machine-primitive.machine-start.') {
                machineProcessing[chunk.parameters.machine_uuid.uuid] = chunk.parameters.machine_uuid.process_time;
            } else if (chunk.type === 'node.primitive.machine-primitive.machine-stop.') {
                const machine = chunk.parameters.machine_uuid;
                machine.outputData.forEach(outputObj => {
                    prevTfs.tfs[outputObj.placeholder.uuid] = frameFromRegion(outputObj.region)
                    if (!activePlaceholders.includes(outputObj.placeholder.uuid)) {
                        activePlaceholders.push(outputObj.placeholder.uuid)
                    }
                })
            } else if (chunk.type === 'node.primitive.machine-primitive.machine-wait.') {
                if (machineProcessing[chunk.parameters.machine_uuid.uuid]) {
                    duration = machineProcessing[chunk.parameters.machine_uuid.uuid]
                }
                steps.push(prevTfs)
            } else if (chunk.type === 'node.primitive.move-trajectory.') {
                duration = chunk.parameters.trajectory_uuid.trace.duration * 1000;
                for (let i = 0; i < chunk.parameters.trajectory_uuid.trace.time_data.length - 1; i++) {
                    prevTfs.tfs = { ...prevTfs.tfs, ...robotFramesFromIdx(i, chunk.parameters.trajectory_uuid.trace) };
                    if (carriedPlaceholder) {
                        // Attach the carried object
                        prevTfs.tfs[carriedPlaceholder] = prevTfs.tfs.simulated_tool0
                    }
                    prevTfs.time = currentTime + chunk.parameters.trajectory_uuid.trace.time_data[i] * 1000;
                    steps.push(prevTfs)
                    prevTfs = lodash.cloneDeep(steps[steps.length - 1]);
                }
                if (chunk.parameters.trajectory_uuid.trace.in_timeout) {
                    cancelled = true
                }
            } else if (chunk.type === 'node.primitive.move-unplanned.') {
                duration = 100;
                prevTfs.time = prevTfs.time + 100;
                prevTfs.tfs = { ...prevTfs.tfs, ...robotFramesFromPose(chunk.parameters.location_uuid) }
                if (carriedPlaceholder) {
                    // Attach the carried object
                    prevTfs.tfs[carriedPlaceholder] = prevTfs.tfs.simulated_tool0
                }
                steps.push(prevTfs)
                prevTfs = lodash.cloneDeep(steps[steps.length - 1]);
            }
        }
        currentTime += duration;
        machineProcessing = objectMap(machineProcessing, val => {
            if (val - duration < 0) {
                return 0
            } else {
                return val - duration
            };
        })
    })
    steps.push({ ...lodash.cloneDeep(steps[steps.length - 1]), time: currentTime + 1000 })
    return stepsToAnimatedTfs(steps)
}

function interpolateScalar(x, y) {
    //const defaultFn = (v) => 0;
    if (x.length <= 0) {
        return null
    }
    const interp = (v) => {
        const val = v > x[x.length - 1] ? v % x[x.length - 1] : v;
        let lastIdx = 0;
        for (let i = 0; i < x.length; i++) {
            if (x[i] <= val) {
                lastIdx = i
            } else {
                break
            }
        }
        return y[lastIdx]
    }
    return interp
}

export function unFlattenProgramPrimitives(primitives, ids) {
    let unFlattenedPrimitives = [];
    ids.forEach(id => {
        let newPrimitive = lodash.omit(primitives[id], 'children');
        if (newPrimitive.type.includes('hierarchical')) {
            newPrimitive.primitives = unFlattenProgramPrimitives(primitives, primitives[id].children);
        };
        unFlattenedPrimitives.push(newPrimitive);
    });
    return unFlattenedPrimitives;
}

export function unFlattenProgramSkills(skills, primitives) {
    let unflattenedSkillSet = Object.values(skills);
    let unFlattenedSkills = [];
    unflattenedSkillSet.forEach(skill => {
        let newSkill = lodash.omit(skill, 'children');
        newSkill.primitives = unFlattenProgramPrimitives(primitives, skill.children);
        unFlattenedSkills.push(newSkill);
    });
    return unFlattenedSkills;
}

export function poseToColor(pose, frame, focused, occupancyZones) {
    let color = { r: 255, g: 255, b: 255, a: focused ? 1 : 0 };
    let pos = pose.refData ? pose.refData.properties.position : pose.properties.position;
    if (frame === 'safety' && occupancyOverlap(pos, occupancyZones)) {
        color.r = 233;
        color.g = 53;
        color.b = 152;
    } else if (frame === 'performance' && !pose.reachable) {
        color.r = 204;
        color.g = 75;
        color.b = 10;
    }
    return color
}

export function poseDataToShapes(pose, frame, occupancyZones) {
    let pose_stored = pose;
    return [
        {
            uuid: `${pose_stored.id}-tag`,
            frame: 'world',
            name: pose.name,
            shape: pose_stored.type.includes('location') ? 'flag' : 'tag',
            position: pose_stored.refData ? pose_stored.refData.properties.position : pose_stored.properties.position,
            rotation: { w: 1, x: 0, y: 0, z: 0 },
            scale: { x: -0.25, y: 0.25, z: 0.25 },
            highlighted: false,
            showName: false,
            color: poseToColor(pose_stored, frame, false, occupancyZones)
        },
        {
            uuid: `${pose_stored.id}-pointer`,
            frame: 'world',
            shape: pose_stored.type.includes('location') ? 'package://app/meshes/LocationMarker.stl' : 'package://app/meshes/OpenWaypointMarker.stl',
            position: pose_stored.refData ? pose_stored.refData.properties.position : pose_stored.properties.position,
            rotation: pose_stored.refData ? pose_stored.refData.properties.rotation : pose_stored.properties.rotation,
            scale: { x: 1, y: 1, z: 1 },
            highlighted: false,
            showName: false,
            color: poseToColor(pose_stored, frame, false, occupancyZones)
        }
    ]
}

export function trajectoryDataToLine(trajectory, locations, waypoints, frame, reachableAndPerformance) {
    // For the time being, enumerate the location and waypoints //197, 50, 154
    let points = [];
    if (trajectory.start_location_uuid) {
        let location = locations[trajectory.start_location_uuid];
        let position = { x: location.position.x, y: location.position.y, z: location.position.z };

        if ((frame === 'performance' && location.joints.reachable) || frame === 'quality' || frame === 'business') {

            points.push({ position, color: { r: 209, g: 0, b: 146, a: 1 } });

        } else {
            points.push({ position, color: poseToColor(location, frame, true) });
        }


    }
    trajectory.waypoint_uuids.forEach(waypoint_uuid => {
        let waypoint = waypoints[waypoint_uuid];
        let position = { x: waypoint.position.x, y: waypoint.position.y, z: waypoint.position.z };
        if ((frame === 'performance' && waypoint.joints.reachable) || frame === 'quality' || frame === 'business') {
            points.push({ position, color: { r: 209, g: 0, b: 146, a: 1 } })
        } else {
            points.push({ position, color: poseToColor(waypoint, frame, true) })
        }

    })

    if (trajectory.end_location_uuid) {
        let location = locations[trajectory.end_location_uuid];
        let position = { x: location.position.x, y: location.position.y, z: location.position.z };
        if ((frame === 'performance' && location.joints.reachable) || frame === 'quality' || frame === 'business') {
            points.push({ position, color: { r: 209, g: 0, b: 146, a: 1 } });
        } else {
            points.push({ position, color: poseToColor(location, frame, true) });
        }
    }
    return {
        name: trajectory.name,
        frame: "world",
        width: 0,
        vertices: points
    }
}

export const machineDataToPlaceholderPreviews = (machine, things, regions) => {
    let items = {};
    Object.keys(machine.inputs).forEach(thingType => {
        const thingInfo = things[thingType];
        machine.inputs[thingType].forEach(zoneInfo => {
            const region = regions[zoneInfo.region_uuid];
            items[thingType + region.uuid] = {
                shape: EVD_MESH_LOOKUP[thingInfo.mesh_id],
                frame: region.uuid,
                position: { x: 0, y: 0, z: 0 },
                rotation: { w: 1, x: 0, y: 0, z: 0 },
                scale: { x: 0.2, y: 0.2, z: 0.2 },
                transformMode: 'inactive',
                color: { r: 200, g: 0, b: 0, a: 0.2 },
                highlighted: false,
                hidden: false,
                onClick: (_) => { }
            }

        })

    })
    Object.keys(machine.outputs).forEach(thingType => {
        const thingInfo = things[thingType];
        machine.outputs[thingType].forEach(zoneInfo => {
            const region = regions[zoneInfo.region_uuid];
            items[thingType + region.uuid] = {
                shape: EVD_MESH_LOOKUP[thingInfo.mesh_id],
                frame: region.uuid,
                position: { x: 0, y: 0, z: 0 },
                rotation: { w: 1, x: 0, y: 0, z: 0 },
                scale: { x: 0.2, y: 0.2, z: 0.2 },
                transformMode: 'inactive',
                color: { r: 0, g: 200, b: 0, a: 0.2 },
                highlighted: false,
                hidden: false,
                onClick: (_) => { }
            }
        })
    })

    return items
}

export const traceToEEPoseScores = (trace) => {
    let scores = [0];
    for (let i = 1; i < trace.frames['tool0'].length; i++) {
        const p1 = trace.frames['tool0'][i][0];
        const p0 = trace.frames['tool0'][i - 1][0];
        const q1 = trace.frames['tool0_endpoint'][i][0];
        const movementVec = new Vector3(p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]);
        const directionVec = new Vector3(q1[0] - p1[0], q1[1] - p1[1], q1[2] - p1[2]);
        scores.push(1000 * movementVec.manhattanLength() / Math.pow(Math.E, 10 * movementVec.angleTo(directionVec)))

    }
    return scores
}

export const traceToVertices = (trace) => {
    let vertices = [];
    ROBOT_FRAMES.forEach(frame => {
        for (let i = 0; i < trace.frames[frame].length; i += 10) {
            vertices.push(new Vector3(...trace.frames[frame][i][0]))
        }
    })
    return vertices
}

export const verticesToVolume = (vertices) => {
    let geometry = new ConvexGeometry(vertices);
    return getVolume(geometry)
}

export const spaceEstimate = (trace) => {
    let geometry = new ConvexGeometry(traceToVertices(trace));
    return getVolume(geometry)
}

/*
https://discourse.threejs.org/t/volume-of-three-buffergeometry/5109
*/
function getVolume(geometry) {
    if (!geometry.isBufferGeometry) {
        console.log("'geometry' must be an indexed or non-indexed buffer geometry");
        return 0;
    }
    var isIndexed = geometry.index !== null;
    let position = geometry.attributes.position;
    let sum = 0;
    let p1 = new Vector3(),
        p2 = new Vector3(),
        p3 = new Vector3();
    if (!isIndexed) {
        let faces = position.count / 3;
        for (let i = 0; i < faces; i++) {
            p1.fromBufferAttribute(position, i * 3 + 0);
            p2.fromBufferAttribute(position, i * 3 + 1);
            p3.fromBufferAttribute(position, i * 3 + 2);
            sum += signedVolumeOfTriangle(p1, p2, p3);
        }
    }
    else {
        let index = geometry.index;
        let faces = index.count / 3;
        for (let i = 0; i < faces; i++) {
            p1.fromBufferAttribute(position, index.array[i * 3 + 0]);
            p2.fromBufferAttribute(position, index.array[i * 3 + 1]);
            p3.fromBufferAttribute(position, index.array[i * 3 + 2]);
            sum += signedVolumeOfTriangle(p1, p2, p3);
        }
    }
    return sum;
}

function signedVolumeOfTriangle(p1, p2, p3) {
    return p1.dot(p2.cross(p3)) / 6.0;
}

export function idleTimeEstimate(unrolled) {
    let delay = 0;
    let gripperStart = 50;
    let gripperEnd = null;
    let first = true;
    let tasks = [];
    if (unrolled === undefined || unrolled === null) {
        return 0;
    } else {
        Object.values(unrolled).forEach(primitive => {
            //console.log(primitive);
            if (primitive.type === 'node.primitive.breakpoint.') {
                return delay;

            } else if (primitive.type === 'node.primitive.delay.') {
                if (tasks.length === 0) {
                    delay += primitive.parameters.duration;
                } else {
                    let assigned = false;
                    let i = 0;
                    let waiting = false;
                    while (assigned === false && i < tasks.length) {
                        if (tasks[i].status === 'started') {
                            tasks[i].timeTaken += primitive.parameters.duration;
                            delay += primitive.parameters.duration;
                            assigned = true;
                        } else if (tasks[i].status === 'waiting') {
                            waiting = true;
                            assigned = true;
                        } else {
                            i += 1;
                        }
                    }
                    if (i === tasks.length || waiting === false) {
                        delay += primitive.parameters.duration;
                    }
                }


            } else if (primitive.type === 'node.primitive.gripper.') {
                if (tasks.length === 0) {
                    if (first === true) {
                        //gripperStart = initialTrajectory;
                        gripperEnd = primitive.parameters.position;
                        first = false;
                        //delay += Math.abs((gripperEnd - gripperStart) / primitive.parameters.speed);
                        gripperStart = primitive.parameters.position;

                    } else {
                        gripperEnd = primitive.parameters.position;
                        //delay += Math.abs((gripperEnd - gripperStart) / primitive.parameters.speed);
                        gripperStart = primitive.parameters.position;
                    }
                } else {
                    let assigned = false;
                    let i = 0;
                    let waiting = false;
                    while (assigned === false && i < tasks.length) {
                        if (tasks[i].status === 'started') {
                            if (first === true) {
                                //gripperStart = initialTrajectory;
                                gripperEnd = primitive.parameters.position;
                                first = false;
                                //delay += Math.abs((gripperEnd - gripperStart) / primitive.parameters.speed);
                                gripperStart = primitive.parameters.position;
                                tasks[i].timeTaken += Math.abs((gripperEnd - gripperStart) / primitive.parameters.speed);

                            } else {
                                gripperEnd = primitive.parameters.position;
                                //delay += Math.abs((gripperEnd - gripperStart) / primitive.parameters.speed);
                                gripperStart = primitive.parameters.position;
                                tasks[i].timeTaken += Math.abs((gripperEnd - gripperStart) / primitive.parameters.speed);
                            }
                            assigned = true;
                        } else if (tasks[i].status === 'waiting') {
                            waiting = true;
                            assigned = true;
                        } else {
                            i += 1;
                        }
                    }
                    if (i === tasks.length || waiting === false) {
                        gripperEnd = primitive.parameters.position;
                        //delay += Math.abs((gripperEnd - gripperStart) / primitive.parameters.speed);
                        gripperStart = primitive.parameters.position;
                    }
                }
            } else if (primitive.type === 'node.primitive.move-trajectory.') {
                if (primitive.trajectory_uuid === null) {
                    delay += 0;
                } else {
                    if (tasks.length === 0) {
                        delay += primitive.parameters.trajectory_uuid.trace.duration;
                    } else {
                        let assigned = false;
                        let i = 0;
                        let waiting = true;
                        while (assigned === false && i < tasks.length) {
                            if (tasks[i].status === 'started') {
                                tasks[i].timeTaken += primitive.parameters.trajectory_uuid.trace.duration;
                                //delay += primitive.parameters.trajectory_uuid.trace.duration ;
                                assigned = true;
                            } else if (tasks[i].status === 'waiting') {
                                waiting = true;
                                assigned = true;
                            } else {
                                i += 1;
                            }
                        }
                        if (i === tasks.length || waiting === false) {
                            // delay += primitive.parameters.trajectory_uuid.trace.duration ;                            
                        }
                    }
                }
            } else if (primitive.type === 'node.primitive.machine-primitive.machine-start.') {
                tasks.push({
                    'machineUUID': primitive.parameters.machine_uuid.uuid,
                    'status': 'started', 'processTime': primitive.parameters.machine_uuid.process_time, 'timeTaken': 0
                });

            } else if (primitive.type === 'node.primitive.machine-primitive.machine-wait.') {
                if (tasks.length === 0) {
                    // console.log("this might be an error");
                } else {

                    tasks.forEach((task) => {

                        if (primitive.parameters.machine_uuid.uuid === task.machineUUID && task.status === 'started') {
                            delay += task.processTime - task.timeTaken;
                            //console.log(duration);
                            task.timeTaken += task.processTime - task.timeTaken;
                            // console.log(task.timeTaken);    
                            task.status = 'waiting';

                        }
                    })
                }
            } else if (primitive.type === 'node.primitive.machine-primitive.machine-stop.') {
                tasks.forEach((task) => { if (primitive.parameters.machine_uuid.uuid === task.machineUUID) task.status = 'stopped' });
            }
        });

        tasks.forEach((task) => {
            if (task.status === 'closed') {

            }
        })
        return delay;
    }
}

export function durationEstimate(unrolled) {
    let duration = 0;
    let gripperStart = 50;
    let gripperEnd = null;
    let first = true;
    let tasks = [];
    // console.log("this is :" + unrolled);
    if (unrolled === undefined || unrolled === null) {
        return 0;
    } else {
        Object.values(unrolled).forEach(primitive => {
            //console.log(primitive);
            if (primitive.type === 'node.primitive.breakpoint.') {
                return duration;

            } else if (primitive.type === 'node.primitive.delay.') {
                if (tasks.length === 0) {
                    duration += primitive.parameters.duration;
                } else {
                    let assigned = false;
                    let i = 0;
                    let waiting = false;
                    while (assigned === false && i < tasks.length) {
                        if (tasks[i].status === 'started') {
                            tasks[i].timeTaken += primitive.parameters.duration;
                            duration += primitive.parameters.duration;
                            assigned = true;
                        } else if (tasks[i].status === 'waiting') {
                            waiting = true;
                            assigned = true;
                        } else {
                            i += 1;
                        }
                    }
                    if (i === tasks.length || waiting === false) {
                        duration += primitive.parameters.duration;
                    }
                }


            } else if (primitive.type === 'node.primitive.gripper.') {
                if (tasks.length === 0) {
                    if (first === true) {
                        //gripperStart = initialTrajectory;
                        gripperEnd = primitive.parameters.position;
                        first = false;
                        duration += Math.abs((gripperEnd - gripperStart) / primitive.parameters.speed);
                        gripperStart = primitive.parameters.position;

                    } else {
                        gripperEnd = primitive.parameters.position;
                        duration += Math.abs((gripperEnd - gripperStart) / primitive.parameters.speed);
                        gripperStart = primitive.parameters.position;
                    }
                } else {
                    let assigned = false;
                    let i = 0;
                    let waiting = false;
                    while (assigned === false && i < tasks.length) {
                        if (tasks[i].status === 'started') {
                            if (first === true) {
                                //gripperStart = initialTrajectory;
                                gripperEnd = primitive.parameters.position;
                                first = false;
                                duration += Math.abs((gripperEnd - gripperStart) / primitive.parameters.speed);
                                gripperStart = primitive.parameters.position;
                                tasks[i].timeTaken += Math.abs((gripperEnd - gripperStart) / primitive.parameters.speed);

                            } else {
                                gripperEnd = primitive.parameters.position;
                                duration += Math.abs((gripperEnd - gripperStart) / primitive.parameters.speed);
                                gripperStart = primitive.parameters.position;
                                tasks[i].timeTaken += Math.abs((gripperEnd - gripperStart) / primitive.parameters.speed);
                            }
                            assigned = true;
                        } else if (tasks[i].status === 'waiting') {
                            waiting = true;
                            assigned = true;
                        } else {
                            i += 1;
                        }
                    }
                    if (i === tasks.length || waiting === false) {
                        gripperEnd = primitive.parameters.position;
                        duration += Math.abs((gripperEnd - gripperStart) / primitive.parameters.speed);
                        gripperStart = primitive.parameters.position;
                    }
                }
            } else if (primitive.type === 'node.primitive.move-trajectory.') {
                if (primitive.trajectory_uuid === null) {
                    duration += 0;
                } else {
                    if (tasks.length === 0) {
                        duration += primitive.parameters.trajectory_uuid.trace.duration;
                    } else {
                        let assigned = false;
                        let i = 0;
                        let waiting = true;
                        while (assigned === false && i < tasks.length) {
                            if (tasks[i].status === 'started') {
                                tasks[i].timeTaken += primitive.parameters.trajectory_uuid.trace.duration;
                                duration += primitive.parameters.trajectory_uuid.trace.duration;
                                assigned = true;
                            } else if (tasks[i].status === 'waiting') {
                                waiting = true;
                                assigned = true;
                            } else {
                                i += 1;
                            }
                        }
                        if (i === tasks.length || waiting === false) {
                            duration += primitive.parameters.trajectory_uuid.trace.duration;
                        }
                    }
                }
            } else if (primitive.type === 'node.primitive.machine-primitive.machine-start.') {
                tasks.push({
                    'machineUUID': primitive.parameters.machine_uuid.uuid,
                    'status': 'started', 'processTime': primitive.parameters.machine_uuid.process_time, 'timeTaken': 0
                });

            } else if (primitive.type === 'node.primitive.machine-primitive.machine-wait.') {
                if (tasks.length === 0) {
                    // console.log("this might be an error");
                } else {

                    tasks.forEach((task) => {

                        if (primitive.parameters.machine_uuid.uuid === task.machineUUID && task.status === 'started') {
                            duration += task.processTime - task.timeTaken;
                            // console.log(duration);
                            task.timeTaken += task.processTime - task.timeTaken;
                            // console.log(task.timeTaken);    
                            task.status = 'waiting';

                        }
                    })
                }
            } else if (primitive.type === 'node.primitive.machine-primitive.machine-stop.') {
                tasks.forEach((task) => { if (primitive.parameters.machine_uuid.uuid === task.machineUUID) task.status = 'stopped' });
            }
        });

        tasks.forEach((task) => {
            if (task.status === 'closed') {

            }
        })
        return duration;
    }
}

function distanceBetween(p1, p2) {
    return Math.sqrt(Math.pow(p1.x - p2.x, 2) + Math.pow(p1.y - p2.y, 2));
}

function isViewRect(entry) {
    return 'top' in entry;
}

function cornersOfRectangle(
    rect,
    left = rect.offsetLeft,
    top = rect.offsetTop,
    transform
) {
    return [
        {
            x: left - (transform.x) / transform.zoom,
            y: top - (transform.y) / transform.zoom,
        },
        {
            x: left + (rect.width - transform.x) / transform.zoom,
            y: top - (transform.y) / transform.zoom,
        },
        {
            x: left - (transform.x) / transform.zoom,
            y: top + (rect.height - transform.y) / transform.zoom,
        },
        {
            x: left + (rect.width - transform.x) / transform.zoom,
            y: top + (rect.height - transform.y) / transform.zoom,
        },
    ];
}



export const thresholdedClosestCorners = ({
    collisionRect,
    droppableContainers,
    editorTransform,
}) => {
    //console.log(editorTransform)
    let minDistanceToCorners = Infinity;
    let minDistanceContainer = null;
    const corners = cornersOfRectangle(
        collisionRect,
        collisionRect.left,
        collisionRect.top,
        editorTransform
    );

    for (const droppableContainer of droppableContainers) {
        const {
            rect: { current: rect },
        } = droppableContainer;

        if (rect) {
            const rectCorners = cornersOfRectangle(
                rect,
                isViewRect(rect) ? rect.left : undefined,
                isViewRect(rect) ? rect.top : undefined,
                { x: 0, y: 0, zoom: 1 }
            );
            const distances = corners.reduce((accumulator, corner, index) => {
                return accumulator + distanceBetween(rectCorners[index], corner);
            }, 0);
            const effectiveDistance = Number((distances / 4).toFixed(4));

            if (effectiveDistance < minDistanceToCorners) {
                minDistanceToCorners = effectiveDistance;
                minDistanceContainer = droppableContainer.id;
            }
        }
    }
    return minDistanceContainer;
};