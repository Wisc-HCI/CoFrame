import {useSceneStore} from 'robot-scene';

const ACTIVE_TFS = [
  'simulated_base_link',
  'simulated_shoulder_link',
  'simulated_upper_arm_link',
  'simulated_forearm_link',
  'simulated_wrist_1_link',
  'simulated_wrist_2_link',
  'simulated_wrist_3_link'
]

export const SimSlice = (set,get) => ({
    updateInterval:null,
    tfs:{},
    staticScene: {},
    animationObjects: {},
    controllers: {},
    updateFromTfs: (msg) => set(state=>{
        msg.transforms.forEach(transformstamped=>{
            if (ACTIVE_TFS.indexOf(transformstamped.child_frame_id) >= 0) {
                state.tfs[transformstamped.child_frame_id].translation = transformstamped.transform.translation;
                state.tfs[transformstamped.child_frame_id].rotation = transformstamped.transform.rotation;
                useSceneStore.getState().setTf(transformstamped.child_frame_id,get().tfs[transformstamped.child_frame_id])
            }
        })
    }),
    setup: (initial_data)=>set((state)=>{
        state.staticScene = initial_data.staticScene;
        state.tfs = initial_data.tfs;
        let items = {};
        Object.keys(initial_data.staticScene)
            .forEach((itemKey)=>{
                let item = initial_data.staticScene[itemKey];
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
                    onClick: (e) => {e.stopPropagation();console.log(itemKey);get().setFocusItem('scene',itemKey)},
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
                        onClick: (e)=>{e.stopPropagation();get().setFocusItem('scene',itemKey+'-collision')},
                        onMove: (transform) => {console.log(transform)}
                    };
                }
            });
            useSceneStore.getState().setItems(items);
            useSceneStore.getState().setTfs(initial_data.tfs);
            useSceneStore.getState().clearLines();
            state.updateInterval = setInterval(get().sync,100)
    }),
    sync: ()=>(state=>{
        console.log('update')
        Object.keys(state.staticScene)
            .forEach((itemKey)=>{
                let item = get().staticScene[itemKey];
                if (item.pending) {
                    useSceneStore.getState().setItem(itemKey,{
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
                    })
                }
                if (item.pending && item.showCollision) {
                    useSceneStore.getState().setItem(itemKey+'-collision',{
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
                    })
                } else if (item.pending && !item.showCollision) {
                    useSceneStore.getState().removeItem(itemKey+'-collision')
                }
        });
    })
});
