// import { useSceneStore } from "robot-scene";
import { trajectoryDataToLine, poseToColor, poseDataToShapes } from "./helpers";

// const setFocusItem = useStore.getState().setFocusItem;
// const setSecondaryFocusItem = useStore.getState().setSecondaryFocusItem;

export const ANIMATION = {
    INACTIVE: "inactive",
    PLAYING: "playing",
    PAUSED: "paused"
}

export const ACTION = {
    SET_ITEM: "set_item",
    REMOVE_ITEM: "remove_item",
    SET_ITEM_SHOW_NAME: "set_item_show_name",
    SET_ITEM_POSITION: "set_item_position",
    SET_ITEM_ROTATION: "set_item_rotation",
    SET_ITEM_SCALE: "set_item_scale",
    SET_ITEM_COLOR: "set_item_color",
    SET_ITEM_WIREFRAME: "set_item_wireframe",
    SET_ITEM_HIGHLIGHTED: "set_item_highlighted",
    SET_ITEM_ONCLICK: "set_item_onclick",
    SET_ITEM_ONPOINTEROVER: "set_item_onpointerover",
    SET_ITEM_ONPOINTEROUT: "set_item_onpointerout",
    SET_ITEM_TRANSFORM_MODE: "set_item_transform_mode",
    SET_ITEM_ONMOVE: "set_item_onmove",
    
    SET_TF: "set_tf",
    REMOVE_TF: "remove_tf",
    SET_TF_POSITION: "set_tf_position",
    SET_TF_ROTATION: "set_tf_rotation",
    
    SET_HULL: "set_hull",
    REMOVE_HULL: "remove_hull",
    SET_HULL_NAME: "set_tf_name",
    SET_HULL_WIREFRAME: "set_hull_wireframe",
    SET_HULL_VERTICES: "set_hull_vertices",
    SET_HULL_VERTEX: "set_hull_vertex",
    ADD_HULL_VERTEX: "add_hull_vertex",
    REMOVE_HULL_VERTEX: "remove_hull_vertex",
    SET_HULL_COLOR: "set_hull_color",
    SET_HULL_HIGHLIGHTED: "set_hull_highlighted",
    SET_HOVER_ONCLICK: "set_hull_onclick",
    SET_HOVER_ONPOINTEROVER: "set_hull_onpointerover",
    SET_HOVER_ONPOINTEROUT: "set_hull_onpointerout",
    
    SET_LINE: "set_line",
    REMOVE_LINE: "remove_line",
    SET_LINE_NAME: "set_line_name",
    SET_LINE_WIDTH: "set_line_width",
    SET_LINE_VERTICES: "set_line_vertices",
    ADD_LINE_VERTEX: "add_line_vertex",
    REMOVE_LINE_VERTEX: "remove_line_vertex",
    SET_LINE_VERTEX: "set_line_vertex",
}

export const ANIMATIONFN = {
    [ACTION.SET_ITEM]: (store)=>store.setItem,
    [ACTION.REMOVE_ITEM]: (store)=>store.removeItem,
    [ACTION.SET_ITEM_SHOW_NAME]: (store)=>store.setItemShowName,
    [ACTION.SET_ITEM_POSITION]: (store)=>store.setItemPosition,
    [ACTION.SET_ITEM_ROTATION]: (store)=>store.setItemRotation,
    [ACTION.SET_ITEM_SCALE]: (store)=>store.setItemScale,
    [ACTION.SET_ITEM_COLOR]: (store)=>store.setItemColor,
    [ACTION.SET_ITEM_HIGHLIGHTED]: (store)=>store.setItemHighlighted,
    [ACTION.SET_ITEM_ONCLICK]: (store)=>store.setItemOnClick,
    [ACTION.SET_ITEM_ONPOINTEROVER]: (store)=>store.setItemOnPointerOver,
    [ACTION.SET_ITEM_ONPOINTEROUT]: (store)=>store.setItemOnPointerOut,
    [ACTION.SET_ITEM_TRANSFORM_MODE]: (store)=>store.setItemTransformMode,
    [ACTION.SET_ITEM_ONMOVE]: (store)=>store.setItemOnMove,

    [ACTION.SET_TF]: (store)=>store.setTF,
    [ACTION.REMOVE_TF]: (store)=>store.removeTF,
    [ACTION.SET_TF_POSITION]: (store)=>store.setTfPosition,
    [ACTION.SET_TF_ROTATION]: (store)=>store.setTfRotation,

    [ACTION.SET_HULL]: (store)=>store.setHull,
    [ACTION.REMOVE_HULL]: (store)=>store.removeHull,
    [ACTION.SET_HULL_NAME]: (store)=>store.setHullName,
    [ACTION.SET_HULL_VERTICES]: (store)=>store.setHullVertices,
    [ACTION.SET_HULL_VERTEX]: (store)=>store.setHullVertex,
    [ACTION.ADD_HULL_VERTEX]: (store)=>store.addHullVertex,
    [ACTION.REMOVE_HULL_VERTEX]: (store)=>store.removeHullVertex,
    [ACTION.SET_HULL_COLOR]: (store)=>store.setHullColor,
    [ACTION.SET_HULL_HIGHLIGHTED]: (store)=>store.setHullHighlighted,
    [ACTION.SET_HOVER_ONCLICK]: (store)=>store.setHullOnClick,
    [ACTION.SET_HOVER_ONPOINTEROVER]: (store)=>store.setHullOnPointerOver,
    [ACTION.SET_HOVER_ONPOINTEROUT]: (store)=>store.setHullOnPointerOut,

    [ACTION.SET_LINE]: (store)=>store.setLine,
    [ACTION.REMOVE_LINE]: (store)=>store.removeLine,
    [ACTION.SET_LINE_NAME]: (store)=>store.setLineName,
    [ACTION.SET_LINE_VERTICES]: (store)=>store.setLineVertices,
    [ACTION.ADD_LINE_VERTEX]: (store)=>store.addLineVertex,
    [ACTION.REMOVE_LINE_VERTEX]: (store)=>store.removeLineVertex,
    [ACTION.SET_LINE_VERTEX]: (store)=>store.setLinevertex,
}

/* 

All animations have the following attributes:
 - onStart: []changes
 - sequence: []{delay:number,changes:[]changes,next:string|sequence}
 - onEnd: []changes

All changes have the following attributes:
 - action: ACTION
 - data: If REMOVE, the uuid, otherwise the data to add.
*/ 

const createSequence = (arrayOfUpdates,terminal) => {
    if (arrayOfUpdates.length === 0) {
        return terminal
    } else {
        return {...arrayOfUpdates[0],next:createSequence(arrayOfUpdates.slice(1))}
    }
}

export const trajectoryToAnimation = (trajectory,locations,waypoints,frame,get) => {
    // const setItem = state.setItem;
    // const setLine = state.setLine;
    // const setItemColor = state.setItemColor;
    // const setLineWidth = state.setLineWidth;
    const setSecondaryFocusItem = get().setSecondaryFocusItem;
    // let trajectoryShapes = trajectoryDataToShapes(trajectory,locations,waypoints,frame,setSecondaryFocusItem);
    let trajectoryLine = trajectoryDataToLine(trajectory,locations,waypoints,frame);
    let onStart = []; // Array of changes
    let onEnd = []; // Array of changes
    let sequence = [];

    let poses = [];
    if (trajectory.start_location_uuid) {
        poses.push(locations[trajectory.start_location_uuid])
    }
    trajectory.waypoint_uuids.forEach(waypoint_uuid=>{
        poses.push(waypoints[waypoint_uuid])
    })
    if (trajectory.end_location_uuid) {
        poses.push(locations[trajectory.end_location_uuid])
    }

    const active_colors = poses.map(pose=>poseToColor(pose,frame,true));
    const inactive_colors = poses.map(pose=>poseToColor(pose,frame,false));

    // Setup the onStart
    poses.forEach((pose,i)=>{
        // Update the shapes and such for each waypoint
        const type = pose.type.includes("location") ? 'location' : 'waypoint';
        const onClick = (e)=>{e.stopPropagation();setSecondaryFocusItem(type,pose.uuid)};
        const shapePair = poseDataToShapes(pose,frame);
        const pairOne = {...shapePair[0],onClick,color:inactive_colors[i]};
        const pairTwo = {...shapePair[1],onClick,color:{...shapePair[1].color,a:0.3}};
        onStart.push({action:ACTION.SET_ITEM,data:[pose.type+'-tag',pairOne]})
        onStart.push({action:ACTION.SET_ITEM,data:[pose.type+'-pointer',pairTwo]})
    })
    // Update the line to be visible and have the correct shape/properties
    onStart.push({action:ACTION.SET_LINE,data:trajectoryLine});

    // Setup the onEnd
    poses.forEach((pose,i)=>{
        const color = {...inactive_colors[i],a:0};
        onEnd.push({action:ACTION.SET_ITEM_COLOR,data:[pose.uuid+'-tag',color]});
        onEnd.push({action:ACTION.SET_ITEM_COLOR,data:[pose.uuid+'-pointer',color]});
    })
    onEnd.push({action:ACTION.SET_LINE_WIDTH,data:[trajectory.uuid,0]})

    poses.forEach((pose,i)=>{
        let changes = [
            {action:ACTION.SET_ITEM_COLOR,data:[pose.uuid+'-tag',active_colors[i]]},
            {action:ACTION.SET_ITEM_COLOR,data:[pose.uuid+'-pointer',active_colors[i]]}
        ]
        
        let otherIndex = i>0 ? i-1 : poses.length-1;
        let otherColor = inactive_colors[otherIndex];
        changes = [
            ...changes,
            {action:ACTION.SET_ITEM_COLOR,data:[pose.uuid+'-tag',otherColor]},
            {action:ACTION.SET_ITEM_COLOR,data:[pose.uuid+'-pointer',otherColor]}
        ]
            
        sequence.push({
            delay: 2000+i*2000,
            changes
        })
    })
    return {onStart, sequence:createSequence(sequence,'restart'), onEnd}
}

export const poseToAnimation = (pose,frame,get) => {
    const setSecondaryFocusItem = get().setSecondaryFocusItem;
    const type = pose.type.includes('location')?'location':'waypoint';
    const onClick = (e)=>{
        e.stopPropagation();
        setSecondaryFocusItem(type,pose.uuid)
    };
    // Works for waypoints and locations.
    // const setItemColor = store.setItemColor;
    // const setItemHighlighted = store.setItemHighlighted;
    const color = poseToColor(pose,frame,true);
    // console.log(setItemColor)
    let onStart = [
        {action:ACTION.SET_ITEM_COLOR,data:[pose.uuid+'-tag',color]},
        {action:ACTION.SET_ITEM_COLOR,data:[pose.uuid+'-pointer',color]},
        {action:ACTION.SET_ITEM_HIGHLIGHTED,data:[pose.uuid+'-tag',true]},
        {action:ACTION.SET_ITEM_HIGHLIGHTED,data:[pose.uuid+'-pointer',true]},
        {action:ACTION.SET_ITEM_ONCLICK,data:[pose.uuid+'-tag',onClick]},
        {action:ACTION.SET_ITEM_ONCLICK,data:[pose.uuid+'-pointer',onClick]}
    ];
    let sequence = {delay:5000,changes:[],next:'restart'};
    let onEnd = [
        {action:ACTION.SET_ITEM_COLOR,data:[pose.uuid+'-tag',{...color,a:0}]},
        {action:ACTION.SET_ITEM_COLOR,data:[pose.uuid+'-pointer',{...color,a:0}]},
        {action:ACTION.SET_ITEM_HIGHLIGHTED,data:[pose.uuid+'-tag',false]},
        {action:ACTION.SET_ITEM_HIGHLIGHTED,data:[pose.uuid+'-pointer',false]},
        {action:ACTION.SET_ITEM_ONCLICK,data:[pose.uuid+'-tag',()=>{}]},
        {action:ACTION.SET_ITEM_ONCLICK,data:[pose.uuid+'-pointer',()=>{}]}
    ];
    return {onStart,sequence,onEnd}
    
}

export const traceToAnimation = (trajectory,frame) => {

}

export const gripperToAnimation = (primitive) => {

}

