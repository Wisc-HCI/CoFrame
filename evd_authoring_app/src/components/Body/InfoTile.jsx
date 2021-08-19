import React, {useCallback} from 'react';
import { Card } from 'antd';
import useGuiStore from '../../stores/GuiStore';
import useReviewStore from '../../stores/ReviewStore';
import useEvdStore, {typeToKey} from '../../stores/EvdStore';
import { getLocationInfo } from '../ContextualInfo/Locations';
import { getWaypointInfo } from '../ContextualInfo/Waypoints';
import { getMachineInfo } from '../ContextualInfo/Machines';
import { getThingInfo } from '../ContextualInfo/Things';
import { getProgramInfo } from '../ContextualInfo/ProgramBlock';
import { getSkillInfo } from '../ContextualInfo/SkillBlock';
import { getPrimitiveInfo } from '../ContextualInfo/PrimitiveBlock';
import { getTrajectoryInfo } from '../ContextualInfo/TrajectoryBlock';

export function InfoTile(_) {

    const { editorPane, setupTab, focusItem, secondaryFocusItem, frame, primaryColor } = useGuiStore(state=>({
        editorPane:state.editorPane,
        setupTab:state.setupTab,
        focusItem:state.focusItem,
        secondaryFocusItem:state.secondaryFocusItem,
        frame:state.frame,
        primaryColor:state.primaryColor
    }))

    const currentIssue = useReviewStore(state=>(secondaryFocusItem.type==='issue'?state.issues[secondaryFocusItem.uuid]:null));
    const [focusData, secondaryFocusData] = useEvdStore(useCallback(state=>{
        let focusData = null;
        let secondaryFocusData = null;
        if (focusItem.type === 'program') {
            focusData = {uuid:state.uuid,name:state.name,description:state.description}
        } else if (focusItem.type) {
            focusData = state.data[typeToKey(focusItem.type)][focusItem.uuid]
        } 
        if (secondaryFocusItem.type && secondaryFocusItem.type !== "issue") {
            secondaryFocusData = state.data[typeToKey(secondaryFocusItem.type)][secondaryFocusItem.uuid]
        }
        return [focusData,secondaryFocusData]
    },[focusItem,secondaryFocusItem]))

    let title = '';
    let description = <p></p>

    const issueParams = {editorPane,setupTab,frame,primaryColor,focusData,secondaryFocusData,currentIssue}

    if (focusItem.type === 'location' || (editorPane === 'setup' && setupTab === 'locations')) {
        [title, description] = getLocationInfo(issueParams)
    } else if (focusItem.type === 'waypoint' || (editorPane === 'setup' && setupTab === 'waypoints')) {
        [title, description] = getWaypointInfo(issueParams)
    } else if (focusItem.type === 'machine' || (editorPane === 'setup' && setupTab === 'machines')) {
        [title, description] = getMachineInfo(issueParams)
    } else if (focusItem.type === 'thing' || (editorPane === 'setup' && setupTab === 'thingTypes')) {
        [title, description] = getThingInfo(issueParams)
    } else if (focusItem.type === 'program' || editorPane === 'program') {
        [title, description] = getProgramInfo(issueParams)
    } else if (focusItem.type === 'skill') {
        [title, description] = getSkillInfo(issueParams)
    } else if (focusItem.type === 'primitive') {
        [title, description] = getPrimitiveInfo(issueParams)
    } else if (focusItem.type === 'trajectory') {
        [title, description] = getTrajectoryInfo(issueParams)
    } 

    return (
        <Card title={title} style={{flex:1}} bodyStyle={{overflowY:'scroll',height:'calc(100vh - 650pt)'}}>
            {description}
        </Card>
    )

    // if (secondaryFocusItem.type === 'issue' && currentIssue) {
    //     title = 'Info: Issue';
    //     description = currentIssue.description;
    // } else if (editorPane === 'program') {
    //     title = 'Info: The Program Editor';
    //     description = <p>You can adjust the program in the Program Editor by dragging elements from the drawer into the canvas. Elements can be modified by clicking on them from within the canvas.</p>
    // } else if (editorPane === 'setup' && focusItem.type === null) {
    //     if (setupTab === 'locations') {
    //         title = 'Info: Locations';
    //         description = <p>Locations are meaningful positions in the scene. For example, they can be used to define goals for placing or picking up <i>Things</i>, or specifying starting or ending positions for the robot.</p>
    //     } else if (setupTab === 'machines') {
    //         title = 'Info: Machines';
    //         description = <p>Machines are items in the workspace that perform tasks, such as modifying or combining <i>Things</i> after placement in defined configurations. They also encode the duration for each process.</p>
    //     } else if (setupTab === 'waypoints') {
    //         title = 'Info: Waypoints';
    //         if (frame ===  'safety') {
    //             description = (
    //                 <div>
    //                     Waypoints are used to construct <i style={{color:primaryColor}}>Trajectories</i> that the robot follows to complete activities.<br/>
    //                     <Alert showIcon message={'Pay special attention to placing waypoints around the occupancy zone of the human, since this is more likely to result in undesirable conflicts between the human and the robot.'}></Alert>
    //                 </div>
    //             )
                
    //         } else if (frame === 'performance') {
    //             description = (
    //                 <div>
    //                     Waypoints are used to construct <i style={{color:primaryColor}}>Trajectories</i> that the robot follows to complete activities.<br/>
    //                     <Alert showIcon message={'Pay special attention to the sequences of waypoints and where they are relative to one another within a trajectory. Longer trajectories take longer to execute and can contribute to greater space usage.'}></Alert>
    //                 </div>
    //             )
    //         } else {
    //             description = <p>Waypoints are used to construct <i style={{color:primaryColor}}>Trajectories</i> that the robot follows to complete activities.</p>
    //         }
            
    //     } else if (setupTab === 'thingTypes') {

    //     }
    // } else if (focusItem.type !== null) {
    //     title = 'Info: Detail';
    //     description = <p>Specific info for this thing</p>
    // }


    // return (
        
    // )
}
