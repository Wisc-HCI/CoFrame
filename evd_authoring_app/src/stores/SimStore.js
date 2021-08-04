import create from "zustand";
import produce from "immer";
import {useSceneStore} from 'robot-scene';

const immer = (config) => (set, get, api) =>
  config((fn) => set(produce(fn)), get, api);

const store = (set,get) => ({
    // The frame specifies the expert (color) frame
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
        }
    },
    robot: {},
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
        useSceneStore.getState().setItems(items);
        useSceneStore.getState().clearLines();
    }),
});

const useSimStore = create(immer(store));

export default useSimStore;