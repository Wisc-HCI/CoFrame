import React, { useState } from 'react';
import { Box, Button, Text } from 'grommet';
import useStore from '../../stores/Store';
import shallow from 'zustand/shallow';
import { FiChevronRight } from 'react-icons/fi';
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
import actionTypes from '../../stores/typeInfo/action';

export function InfoTile(_) {

    const [
        activeDrawer,
        frame,
        primaryColor,
        focusData,
        activeFocus,
        setActiveFocus
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
            state.activeFocus,
            state.setActiveFocus
        ]
    }, shallow)

    // const [currentTab, setCurrentTab] = useState(focusData.length-1);

    let tabs = focusData.map((focusItem,i)=>{
        if (focusItem.code) {
            // Is an issue
            let contents = <div>ISSUE CONTENT</div>;
            return {title:'Issue',key:focusItem.uuid,contents}
        } else if (focusItem.type !== undefined) {
            let contents = <div>DATA CONTENT</div>;
            if (focusItem.type === 'programType') {
                contents = getProgramInfo({frame, primaryColor, focusData:focusItem})
            } else if (Object.keys(actionTypes).includes(focusItem.type)) {
                contents = getPrimitiveInfo({frame, primaryColor, focusData:focusItem})
            }
            // Is a block/data object
            return {title:focusItem.name,key:focusItem.id,contents}
        } else {
            return {title:'null',key:i,contents:<div>NULL CONTENT</div>}
        }
    })

    let tabIdx = 0;
    tabs.some((tab,i)=>{
        if (tab.key === activeFocus) {
            tabIdx = i;
            return true
        } else {
            return false
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
                        <Box key={i} direction='row' align='center' alignContent='center'>
                            <Button
                                plain
                                label={tab.title}
                                onClick={() => setActiveFocus(tab.key)}
                                margin={{top:'7pt',bottom:'7pt',right:'5pt',left:'5pt'}}
                                style={{color:tab.key===activeFocus?primaryColor:'white'}}
                            />
                            {i < tabs.length - 1 && (
                                <FiChevronRight/>
                            )}
                        </Box>
                        
                    )
                    )}
                </Box>}>
            <div style={{ height: 'calc(100vh - 543pt)', overflowY: 'scroll' }}>
                {tabs[tabIdx] ? tabs[tabIdx].contents : tabs[0].contents}
            </div>
        </Tile>
    )
}
