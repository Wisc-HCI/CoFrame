import { useSceneStore } from "robot-scene";
import { trajectoryDataToLine, trajectoryDataToShapes, poseToColor } from "./helpers";

// const setFocusItem = useStore.getState().setFocusItem;
// const setSecondaryFocusItem = useStore.getState().setSecondaryFocusItem;

const ACTION = {
    SET_ITEM: "set_item",
    REMOVE_ITEM: "remove_item",
    SET_ITEM_SHOW_NAME: "set_item_show_name",
    SET_ITEM_POSITION: "set_item_position",
    SET_ITEM_ROTATION: "set_item_rotation",
    SET_ITEM_SCALE: "set_item_scale",
    SET_ITEM_COLOR: "set_item_color",
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
    SET_LINE_VERTICES: "set_line_vertices",
    ADD_LINE_VERTEX: "add_line_vertex",
    REMOVE_LINE_VERTEX: "remove_line_vertex",
    SET_LINE_VERTEX: "set_line_vertex",
}

const STOREFN = {
    [ACTION.SET_ITEM]: useSceneStore.getState().setItem,
    [ACTION.REMOVE_ITEM]: useSceneStore.getState().removeItem,
    [ACTION.SET_ITEM_SHOW_NAME]: useSceneStore.getState().setItemShowName,
    [ACTION.SET_ITEM_POSITION]: useSceneStore.getState().setItemPosition,
    [ACTION.SET_ITEM_ROTATION]: useSceneStore.getState().setItemRotation,
    [ACTION.SET_ITEM_SCALE]: useSceneStore.getState().setItemScale,
    [ACTION.SET_ITEM_COLOR]: useSceneStore.getState().setItemColor,
    [ACTION.SET_ITEM_HIGHLIGHTED]: useSceneStore.getState().setItemHighlighted,
    [ACTION.SET_ITEM_ONCLICK]: useSceneStore.getState().setItemOnClick,
    [ACTION.SET_ITEM_ONPOINTEROVER]: useSceneStore.getState().setItemOnPointerOver,
    [ACTION.SET_ITEM_ONPOINTEROUT]: useSceneStore.getState().setItemOnPointerOut,
    [ACTION.SET_ITEM_TRANSFORM_MODE]: useSceneStore.getState().setItemTransformMode,
    [ACTION.SET_ITEM_ONMOVE]: useSceneStore.getState().setItemOnMove,

    [ACTION.SET_TF]: useSceneStore.getState().setTF,
    [ACTION.REMOVE_TF]: useSceneStore.getState().removeTF,
    [ACTION.SET_TF_POSITION]: useSceneStore.getState().setTfPosition,
    [ACTION.SET_TF_ROTATION]: useSceneStore.getState().setTfRotation,

    [ACTION.SET_HULL]: useSceneStore.getState().setHull,
    [ACTION.REMOVE_HULL]: useSceneStore.getState().removeHull,
    [ACTION.SET_HULL_NAME]: useSceneStore.getState().setHullName,
    [ACTION.SET_HULL_VERTICES]: useSceneStore.getState().setHullVertices,
    [ACTION.SET_HULL_VERTEX]: useSceneStore.getState().setHullVertex,
    [ACTION.ADD_HULL_VERTEX]: useSceneStore.getState().addHullVertex,
    [ACTION.REMOVE_HULL_VERTEX]: useSceneStore.getState().removeHullVertex,
    [ACTION.SET_HULL_COLOR]: useSceneStore.getState().setHullColor,
    [ACTION.SET_HULL_HIGHLIGHTED]: useSceneStore.getState().setHullHighlighted,
    [ACTION.SET_HOVER_ONCLICK]: useSceneStore.getState().setHullOnClick,
    [ACTION.SET_HOVER_ONPOINTEROVER]: useSceneStore.getState().setHullOnPointerOver,
    [ACTION.SET_HOVER_ONPOINTEROUT]: useSceneStore.getState().setHullOnPointerOut,

    [ACTION.SET_LINE]: useSceneStore.getState().setLine,
    [ACTION.REMOVE_LINE]: useSceneStore.getState().removeLine,
    [ACTION.SET_LINE_NAME]: useSceneStore.getState().setLineName,
    [ACTION.SET_LINE_VERTICES]: useSceneStore.getState().setLineVertices,
    [ACTION.ADD_LINE_VERTEX]: useSceneStore.getState().addLineVertex,
    [ACTION.REMOVE_LINE_VERTEX]: useSceneStore.getState().removeLineVertex,
    [ACTION.SET_LINE_VERTEX]: useSceneStore.getState().setLinevertex,
}

const ANIMATION = {
    QUEUED: "queued",
    PLAYING: "playing",
    PAUSED: "paused",
    FINISHED: "finished"
}

const PHASE = {
    START: "start",
    SEQUENCE: "sequence",
    END: "end"
}

/* 

All animations have the following attributes:
 - onStart: []changes
 - sequence: []updates
 - onEnd: []changes

All updates have the following attributes:
 - time: number (delay in milliseconds)
 - changes: []change

All changes have the following attributes:
 - action: ACTION
 - data: If REMOVE, the uuid, otherwise the data to add.
*/ 

export class Animation {
    constructor(onStart, sequence, onEnd, cycle) {
        this.onStart = onStart;
        this.sequence = sequence;
        this.onEnd = onEnd;
        this.state = {
            animation:ANIMATION.QUEUED,
            phase:PHASE.START,
            index:null
        };
        this.cycle = cycle;
    }

    _advance() {
        // This function is used internally and should not be called from anywhere else. 
        // Use the start, pause, and end functions

        // If the animation is queued, finished, or paused, this is a no-op
        if (this.state.animation === ANIMATION.QUEUED || this.state.animation === ANIMATION.FINISHED || this.state.animation === ANIMATION.PAUSED) {
            return
        }
        
        // Make changes (for now we are console logging)
        if (this.state.phase === PHASE.START) {
            // Enumerate the changes and push them off to the store.
            this.onStart.forEach(change=>{
                STOREFN[change.action](...change.data)
                console.log(change)
            })
        } else if (this.state.phase === PHASE.END) {
            this.onEnd.forEach(change=>{
                STOREFN[change.action](...change.data)
                console.log(change)
            })
        } else {
            this.sequence[this.state.index].changes.forEach(change=>{
                STOREFN[change.action](...change.data)
                console.log(change)
            })
        }
        
        // Update the state for the next round and dispatch if necessary.
        if (this.state.phase === PHASE.START) {
            this.state = {...this.state,phase:PHASE.SEQUENCE,index:0};
            setTimeout(()=>this._advance(),0);
            return
        } else if (this.state.phase === PHASE.END) {
        	this.state = {...this.state,animation:ANIMATION.FINISHED,index:null};
            return
        } else if (this.state.phase === PHASE.SEQUENCE && this.sequence[this.state.index+1]) {
            // Sequence isn't over. Go to next step.
            const next = this.state.index+1;
            this.state = {...this.state,index:next};
            setTimeout(()=>this._advance(),this.sequence[next].time);
            return
        } else if (this.state.phase === PHASE.SEQUENCE && this.cycle) {
            // The sequence is at the end, wrap-around to the first index.
            this.state = {...this.state,index:0};
            setTimeout(()=>this._advance(),this.sequence[0].time);
            return
        } else if (this.state.phase === PHASE.SEQUENCE && !this.cycle) {
            // The sequence is at the end, go to finish.
            this.state = {animation:ANIMATION.PLAYING,phase:PHASE.END,index:null};
            setTimeout(()=>this._advance(),0);
            return
        } 
        
        
    }

    start() {
        if (this.state.animation === ANIMATION.PAUSED) {
            this.state = {...this.state,animation:ANIMATION.PLAYING};
            this._advance();
        } else if (this.state.animation === ANIMATION.QUEUED || this.state.animation === ANIMATION.FINISHED) {
            this.state = {animation:ANIMATION.PLAYING,phase:PHASE.START,index:0};
            this._advance();
        }
        // Ignore if already playing
    }

    pause() {
        this.state = {...this.state,animation:ANIMATION.PAUSED};
    }

    end() {
        this.state = {animation:ANIMATION.PLAYING,phase:PHASE.END,index:0};
    }
}

export const trajectoryToAnimation = (trajectory,locations,waypoints,frame,setSecondaryFocusItem) => {
    let trajectoryShapes = trajectoryDataToShapes(trajectory,locations,waypoints,frame,setSecondaryFocusItem);
    let trajectoryLine = trajectoryDataToLine(trajectory,locations,waypoints,frame);
    let onStart = [];
    trajectoryShapes.forEach(shapeArgs=>{
        onStart.push({action:ACTION.SET_ITEM,data:shapeArgs})
    })
    onStart.push({action:ACTION.SET_LINE,data:trajectoryLine});
    let sequence = [];
    trajectoryShapes.forEach((shapeArgs,i)=>{
        let changes = [{
            action:ACTION.SET_ITEM_COLOR,
            data:[shapeArgs[0],poseToColor(shapeArgs[1],frame,true)]
        }];
        if (i > 0) {
            let prevShapeArgs = trajectoryShapes[i-1];
            changes.push({
                action:ACTION.SET_ITEM_COLOR,
                data:[prevShapeArgs[0],poseToColor(prevShapeArgs[1],frame,false)]
            })
        } else {
            let prevShapeArgs = trajectoryShapes[trajectoryShapes.length-1];
            changes.push({
                action:ACTION.SET_ITEM_COLOR,
                data:[prevShapeArgs[0],poseToColor(prevShapeArgs[1],frame,false)]
            })
        }
        sequence.push({
            time: i*2000,
            changes
        })
    })
    let onEnd = [];
    trajectoryShapes.forEach(shapeArgs=>{
        onEnd.push({action:ACTION.REMOVE_ITEM,data:[shapeArgs[0]]});
        onEnd.push({action:ACTION.REMOVE_LINE,data:[trajectory.uuid]})
    })

    // TODO: Add the tf frames changes associated with the robot.
    return new Animation(onStart,sequence,onEnd,true)
}

export const traceToAnimation = (trajectory,frame) => {

}

export const gripperToAnimation = (primitive) => {

}

