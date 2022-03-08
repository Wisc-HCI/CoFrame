import React, { useState } from 'react';
import { Box, Button, Text } from 'grommet';
import useStore from '../../stores/Store';
import shallow from 'zustand/shallow';
import { getSceneInfo } from '../ContextualInfo/Scene';
import { getLocationInfo } from '../ContextualInfo/Locations';
import { getWaypointInfo } from '../ContextualInfo/Waypoints';
import { getMachineInfo } from '../ContextualInfo/Machines';
import { getThingInfo } from '../ContextualInfo/Things';
import { getProgramInfo } from '../ContextualInfo/ProgramBlock';
import { getSkillInfo } from '../ContextualInfo/SkillBlock';
import { getPrimitiveInfo } from '../ContextualInfo/PrimitiveBlock';
import { getTrajectoryInfo } from '../ContextualInfo/TrajectoryBlock';
import { getPlotInfo } from '../ContextualInfo/Plots';
import Tile from '../Tile';
import { DATA_TYPES } from 'simple-vp';

export function InfoTile(_) {

    const [
        activeDrawer,
        frame,
        primaryColor,
        focusData
    ] = useStore(state => {


        let focusData = state.focus.map(f=>{
            if (state.programData[f]) {
                return state.programData[f]
            } else if (state.issues[f]) {
                return state.issues[f]
            } else {
                return null
            }
        }).filter(d=>d!==null);

        let programData = null;
        Object.values(state.programData).some(d=>{
            if (d.dataType === DATA_TYPES.INSTANCE && d.type === 'programType') {
                programData = d;
                return true;
            } else {
                return false
            }
        })

        if (focusData.length === 0) {
            focusData = [programData]
        }

        return [
            state.activeDrawer !== null ? state.programSpec.drawers[state.activeDrawer].title : null,
            state.frame,
            state.primaryColor,
            focusData,
            programData
        ]
    }, shallow)

    const [currentTab, setCurrentTab] = useState(focusData.length-1);

    let tabs = focusData.map(focus=>{
        if (focus.code) {
            // Is an issue
            return {title:'Issue',contents:<div>ISSUE CONTENT</div>}
        } else if (focus.type !== undefined) {
            // Is a block/data object
            return {title:focus.name,contents:<div>DATA CONTENT</div>}
        } else {
            return {title:'null',contents:<div>NULL CONTENT</div>}
        }
    })

    // const issueParams = { activeDrawer, frame, primaryColor, focusData, secondaryFocusData, currentIssue }
    // if (focusData?.type === 'scene') {
    //     tabs = getSceneInfo(issueParams)
    // } else if (focusData?.type === 'location' || (activeDrawer === 'locations')) {
    //     tabs = getLocationInfo(issueParams)
    // } else if (focusData?.type === 'waypoint' || (activeDrawer === 'waypoints')) {
    //     tabs = getWaypointInfo(issueParams)
    // } else if (focusData?.type === 'machine' || (activeDrawer === 'machines')) {
    //     tabs = getMachineInfo(issueParams)
    // } else if (focusData?.type === 'thing' || (activeDrawer === 'things')) {
    //     tabs = getThingInfo(issueParams)
    // } else if (focusData?.type === 'program' || activeDrawer === null) {
    //     tabs = getProgramInfo(issueParams)
    // } else if (focusData?.type === 'skill') {
    //     tabs = getSkillInfo(issueParams)
    // } else if (focusData?.type === 'primitive') {
    //     tabs = getPrimitiveInfo(issueParams)
    // } else if (focusData?.type === 'trajectory') {
    //     tabs = getTrajectoryInfo(issueParams)
    // }

    // if (currentIssue && currentIssue.graphData) {
    //     let temp = getPlotInfo(issueParams);
    //     tabs = tabs.concat(temp);
    // }

    return (
        <Tile
            style={{display:'flex',flexDirection:'column',height:'100%'}}
            borderWidth={4}
            internalPaddingWidth={10}
            header={
                <Box direction='row'>
                    {tabs.map((tab, i) => (
                        <div key={i}>
                            <Button
                                plain
                                label={tab.title}
                                onClick={() => setCurrentTab(i)}
                                margin={{top:'7pt',bottom:'7pt',right:'5pt',left:'5pt'}}
                                style={{color:i===currentTab?primaryColor:'white'}}
                            />
                            {i < tabs.length - 1 && (
                                <Text>/</Text>
                            )}
                        </div>
                        
                    )
                    )}
                </Box>}>
            <div style={{ height: 'calc(100vh - 543pt)', overflowY: 'scroll' }}>
                {tabs[currentTab] ? tabs[currentTab].contents : tabs[0].contents}
            </div>
        </Tile>
    )
}
