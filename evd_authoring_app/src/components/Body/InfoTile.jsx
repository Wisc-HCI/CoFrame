import React, {useCallback, useState} from 'react';
import { Row, Button } from 'antd';
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

    const [currentTab, setCurrentTab] = useState(0);

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

    let tabs = [
        {
            title: 'No Data',
            contents: <div></div>
        }
    ]

    const issueParams = {editorPane,setupTab,frame,primaryColor,focusData,secondaryFocusData,currentIssue}

    if (focusItem.type === 'location' || (editorPane === 'setup' && setupTab === 'locations')) {
        tabs = getLocationInfo(issueParams)
    } else if (focusItem.type === 'waypoint' || (editorPane === 'setup' && setupTab === 'waypoints')) {
        tabs = getWaypointInfo(issueParams)
    } else if (focusItem.type === 'machine' || (editorPane === 'setup' && setupTab === 'machines')) {
        tabs = getMachineInfo(issueParams)
    } else if (focusItem.type === 'thing' || (editorPane === 'setup' && setupTab === 'thingTypes')) {
        tabs = getThingInfo(issueParams)
    // } else if (focusItem.type === 'program' || editorPane === 'program') {
    //     tabs = getProgramInfo(issueParams)
    // } else if (focusItem.type === 'skill') {
    //     tabs = getSkillInfo(issueParams)
    // } else if (focusItem.type === 'primitive') {
    //     tabs = getPrimitiveInfo(issueParams)
    // } else if (focusItem.type === 'trajectory') {
    //     tabs = getTrajectoryInfo(issueParams)
    } 

    return (
        <div style={{height:'calc(100vh - 494pt)',backgroundColor:'rgba(100,100,100,0.3)', padding: 10, borderRadius: 3}}>
            <Row style={{margin:6}}>
                {tabs.map((tab,i)=>
                    <Button 
                        style={{marginRight:10}}
                        type={i===currentTab || (i===0 && currentTab >= tabs.length) ? 'primary' : 'ghost'} 
                        onClick={()=>setCurrentTab(i)}>{tab.title}
                    </Button>
                )}
            </Row>
            <div style={{height:'calc(100vh - 546pt)',backgroundColor:'rgba(0,0,0,0.6)', marginTop:10, padding: 10, borderRadius: 3, color:'white'}}>
                {tabs[currentTab] ? tabs[currentTab].contents : tabs[0].contents}
            </div>
        </div>
    )
}
