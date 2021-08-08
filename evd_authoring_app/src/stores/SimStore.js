import create from "zustand";
import produce from "immer";
import {useSceneStore} from 'robot-scene';

const immer = (config) => (set, get, api) =>
  config((fn) => set(produce(fn)), get, api);

const store = (set,get) => ({
    tfs:{
        'simulated_base_link_inertia':{
            frame:'world',
            translation: { x: 0, y: 0, z: 0 },
            rotation: { w: 1, x: 0, y: 0, z: 0 },
        },
        'simulated_shoulder_link':{
            frame:'simulated_base_link_inertia',
            translation: { x: 0, y: 0, z: 0.15185 },
            rotation: { w: 0.8236985806242969, x: 0, y: 0, z: 0.5670279078471523 },
        },
        'simulated_upper_arm_link':{
            frame:'simulated_shoulder_link',
            translation: { x: 0, y: 0, z: 0 },
            rotation: { w: 0.6506432546460919, x: 0.650643254779541, y: -0.2768814820684781, z: 0.2768814820116888 }
        },
        // 'simulated_forearm_link':{
        //     frame:'simulated_upper_arm_link',
        //     translation: { x: -0.24355, y: 0, z: 0 },
        //     rotation: { w: 0.5211812959955269, x: 0, y: 0, z: 0.8534459893305628 },
        // }
    },
    staticScene: {
        table: {
            visual: "package://evd_ros_tasks/tasks/3d_printer_machine_tending/models/Table/Table.stl",
            collision: "package://evd_ros_tasks/tasks/3d_printer_machine_tending/models/Table/Table.stl",
            name: "Table",
            frame: "world",
            position: { x: 0, y: 0.36, z: -0.37 },
            rotation: { w: 1, x: 0, y: 0, z: 0 },
            color: {r: 40, g: 40, b: 40, a: 1},
            showCollision: false,
            highlighted: false,
            scale: {x:1,y:1,z:1}
        },
        pedestal: {
            visual: "package://evd_ros_tasks/tasks/3d_printer_machine_tending/models/ur3e-Pedestal/Pedestal.stl",
            collision: "package://evd_ros_tasks/tasks/3d_printer_machine_tending/collision_meshes/Pedestal.stl",
            name: "Pedestal",
            frame: "world",
            position: { x: 0, y: 0, z: -0.38 },
            rotation: { w: 1, x: 0, y: 0, z: 0 },
            color: {r: 50, g: 50, b: 50, a: 1},
            showCollison: false,
            highlighted: false,
            scale: {x:1,y:1,z:1}
        },
        box: {
            visual: "package://evd_ros_tasks/tasks/3d_printer_machine_tending/models/Box/Box.stl",
            collision: "package://evd_ros_tasks/tasks/3d_printer_machine_tending/collision_meshes/Box.stl",
            name: "Box",
            frame: "world",
            position: { x: 0.35, y: 0.35, z: 0.07 },
            rotation: { w: 1, x: 0, y: 0, z: 0 },
            showCollison: false,
            highlighted: false,
            scale: {x:1,y:1,z:1}
        },
        printer: {
            visual: "package://evd_ros_tasks/tasks/3d_printer_machine_tending/models/MK2-Printer/MK2-Printer.stl",
            collision: "package://evd_ros_tasks/tasks/3d_printer_machine_tending/collision_meshes/MK2-Printer.stl",
            name: "3D Printer",
            frame: "world",
            position: { x: -0.28, y: 0.32, z: 0.3 },
            rotation: { w: 1, x: 0, y: 0, z: 0 },
            showCollison: false,
            highlighted: true,
            scale: {x:1,y:1,z:1}
        },
        base: {
            visual: "package://universal_robot/ur_description/meshes/ur3/visual/base.dae",
            collision: "package://universal_robot/ur_description/meshes/ur3/collision/base.stl",
            name: 'Base',
            frame: "simulated_base_link_inertia",
            position: { x: 0, y: 0, z: 0 },
            rotation: { w: 1, x: 0, y: 0, z: 0 },
            scale: {x:1,y:1,z:1},
            showCollison: false,
            highlighted: false
        },
        shoulderLink: {
            visual: "package://universal_robot/ur_description/meshes/ur3/visual/shoulder.dae",
            collision: "package://universal_robot/ur_description/meshes/ur3/collision/shoulder.stl",
            name: 'Shoulder Link',
            frame: "simulated_shoulder_link",
            position: { x: 0, y: 0, z: 0 },
            rotation: { w: 1, x: 0, y: 0, z: 0 },
            scale: {x:1,y:1,z:1},
            showCollison: false,
            highlighted: false,
        },
        upperArmLink: {
            visual: "package://universal_robot/ur_description/meshes/ur3/visual/upperarm.dae",
            collision: "package://universal_robot/ur_description/meshes/ur3/collision/upperarm.stl",
            name: "Upper Arm Link",
            frame: "simulated_upper_arm_link",
            position: { x: 0, y: 0, z: 0 },
            rotation: { w: 1, x: 0, y: 0, z: 0 },
            scale: {x:1,y:1,z:1},
            showCollison: false,
            highlighted: false,
        },
        forearmLink: {
            visual: "package://universal_robot/ur_description/meshes/ur3/visual/forearm.dae",
            collision: "package://universal_robot/ur_description/meshes/ur3/collision/forearm.stl",
            name: "Forearm Link",
            frame: "simulated_forearm_link",
            position: { x: 0, y: 0, z: 0 },
            rotation: { w: 1, x: 0, y: 0, z: 0 },
            scale: {x:1,y:1,z:1},
            showCollison: false,
            highlighted: false,
        },
        wrist1Link: {
            visual: "package://universal_robot/ur_description/meshes/ur3/visual/wrist1.dae",
            collision: "package://universal_robot/ur_description/meshes/ur3/collision/wrist1.stl",
            name: "Wrist 1 Link",
            frame: "simulated_wrist_1_link",
            position: { x: 0, y: 0, z: 0 },
            rotation: { w: 1, x: 0, y: 0, z: 0 },
            scale: {x:1,y:1,z:1},
            showCollison: false,
            highlighted: false,
        },
        wrist2Link: {
            visual: "package://universal_robot/ur_description/meshes/ur3/visual/wrist2.dae",
            collision: "package://universal_robot/ur_description/meshes/ur3/collision/wrist2.stl",
            name: "Wrist 2 Link",
            frame: "simulated_wrist_2_link",
            position: { x: 0, y: 0, z: 0 },
            rotation: { w: 1, x: 0, y: 0, z: 0 },
            scale: {x:1,y:1,z:1},
            showCollison: false,
            highlighted: false,
        },
        wrist3Link: {
            visual: "package://universal_robot/ur_description/meshes/ur3/visual/wrist3.dae",
            collision: "package://universal_robot/ur_description/meshes/ur3/collision/wrist3.stl",
            name: "Wrist 3 Link",
            frame: "simulated_wrist_3_link",
            position: { x: 0, y: 0, z: 0 },
            rotation: { w: 1, x: 0, y: 0, z: 0 },
            scale: {x:1,y:1,z:1},
            showCollison: false,
            highlighted: false,
        }
    },
    setup: ()=>set((state)=>{
        let items = {};
        Object.keys(state.staticScene)
            .forEach((itemKey)=>{
                let item = get().staticScene[itemKey];
                items[itemKey] = {
                    shape: item.visual,
                    name: item.name,
                    frame: item.frame,
                    position: item.position,
                    rotation: item.rotation,
                    color: item.color,
                    scale: item.scale,
                    transformMode: 'inactive',
                    highlighted: item.highlighted,
                    onClick: () => {console.log(itemKey)},
                    onMove: (transform) => {console.log(transform)}
                };
                if (item.showCollision) {
                    items[itemKey+'-collision'] = {
                        shape: item.collision,
                        name: item.name+' Collision',
                        frame: item.frame,
                        position: item.position,
                        rotation: item.rotation,
                        scale: { x: 1, y: 1, z: 1 },
                        color: {r: 250, g: 50, b: 50, a: 0.5},
                        transformMode: 'inactive',
                        highlighted: false,
                        onClick: () => {console.log(itemKey)},
                        onMove: (transform) => {console.log(transform)}
                    };
                }
            })
        console.log(items);
        console.log(get().tfs)
        useSceneStore.getState().setItems(items);
        useSceneStore.getState().setTfs(get().tfs);
        useSceneStore.getState().clearLines();
    }),
});

const useSimStore = create(immer(store));

export default useSimStore;