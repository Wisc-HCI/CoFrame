import create from "zustand";
import produce from "immer";
import {useSceneStore} from 'robot-scene';

const immer = (config) => (set, get, api) =>
  config((fn) => set(produce(fn)), get, api);

const store = (set,get) => ({
    // The frame specifies the expert (color) frame
    staticScene: {
        table: {
            visual: "package://app/meshes/description/app/models/Table/Table.fbx",
            collision: "package://app/meshes/Collision-Table.stl",
            name: "Table",
            frame: "world",
            position: { x: 0, y: 0.36, z: -0.37 }, 
            rotation: { w: 1, x: 0, y: 0, z: 0 },
            showCollision: false
        },
        pedestal: {
            visual: "package://app/meshes/description/app/models/ur3e-Pedestal/Pedestal.fbx",
            collision: "package://app/meshes/Collision-Pedestal.stl",
            name: "Pedestal",
            frame: "world",
            position: { x: 0, y: -0.36, z: 0 },
            rotation: { w: 1, x: 0, y: 0, z: 0 },
            showCollison: false
        },
        box: {
            visual: "package://app/meshes/description/app/models/Box/Box.fbx",
            collision: "package://app/meshes/description/app/collision/Collision-Box.stl",
            name: "Box",
            frame: "world",
            position: { x: 0.35, y: 0.35, z: 0.07 },
            rotation: { w: 1, x: 0, y: 0, z: 0 },
            showCollison: true
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
                    scale: { x: 1, y: 1, z: 1 },
                    editMode: 'inactive',
                    highlighted: false,
                    onClick: () => {console.log(itemKey)},
                    onTransform: (transform) => {console.log(transform)}
                };
                if (item.showCollision) {
                    items[itemKey+'-collision'] = {
                        shape: item.collision,
                        name: item.name+' Collision',
                        frame: item.frame,
                        position: item.position,
                        rotation: item.rotation,
                        color: item.color,
                        scale: { x: 1.1, y: 1.1, z: 1.1 },
                        color: {r: 250, g: 50, b: 50, a: 0.5},
                        editMode: 'inactive',
                        highlighted: false,
                        onClick: () => {console.log(itemKey)},
                        onTransform: (transform) => {console.log(transform)}
                    };
                }
            })
        console.log(items);
        useSceneStore.getState().setTfs(
            {
                world: {
                    name: 'world',
                    translation: { x: 0, y: 0, z: 0 },
                    rotation: { w: 1, x: 0, y: 0, z: 0 }
                }
            }
        );
        useSceneStore.getState().setItems(items);
        useSceneStore.getState().clearLines();
    }),
});

const useSimStore = create(immer(store));

export default useSimStore;