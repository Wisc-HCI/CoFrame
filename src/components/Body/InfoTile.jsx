import React, { useState } from 'react';
import { Row, Button } from 'antd';
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

export function InfoTile(_) {

    const [
        focusItem,
        activeDrawer,
        frame,
        primaryColor,
        focusData,
        secondaryFocusData,
        currentIssue
    ] = useStore(state => {

        let focusData = null;
        let secondaryFocusData = null;
        let currentIssue = null;
        if (state.focusItem.type === 'scene') {
            focusData = { uuid: state.focusItem.uuid, type: state.focusItem.type }
        } else if (state.focusItem.type === 'data') {
            focusData = state.data[state.focusItem.uuid]
        }
        if (state.secondaryFocusItem.type && state.secondaryFocusItem.type === "data") {
            secondaryFocusData = state.data[state.secondaryFocusItem.uuid]
        } else if (state.secondaryFocusItem.type === "issue") {
            currentIssue = state.issues[state.secondaryFocusItem.uuid]
        }

        return [
            state.focusItem,
            state.activeDrawer,
            state.frame,
            state.primaryColor,
            focusData,
            secondaryFocusData,
            currentIssue
        ]
    }, shallow)

    const [currentTab, setCurrentTab] = useState(0);

    let tabs = [
        {
            title: 'No Data',
            contents: <div></div>
        }
    ]

    const issueParams = { activeDrawer, frame, primaryColor, focusData, secondaryFocusData, currentIssue }
    if (focusData?.type === 'scene') {
        tabs = getSceneInfo(issueParams)
    } else if (focusData?.type === 'location' || (activeDrawer === 'locations')) {
        tabs = getLocationInfo(issueParams)
    } else if (focusData?.type === 'waypoint' || (activeDrawer === 'waypoints')) {
        tabs = getWaypointInfo(issueParams)
    } else if (focusData?.type === 'machine' || (activeDrawer === 'machines')) {
        tabs = getMachineInfo(issueParams)
    } else if (focusData?.type === 'thing' || (activeDrawer === 'things')) {
        tabs = getThingInfo(issueParams)
    } else if (focusData?.type === 'program' || activeDrawer === null) {
        tabs = getProgramInfo(issueParams)
    } else if (focusData?.type === 'skill') {
        tabs = getSkillInfo(issueParams)
    } else if (focusData?.type === 'primitive') {
        tabs = getPrimitiveInfo(issueParams)
    } else if (focusData?.type === 'trajectory') {
        tabs = getTrajectoryInfo(issueParams)
    }

    if (currentIssue && currentIssue.graphData) {
        let temp = getPlotInfo(issueParams);
        tabs = tabs.concat(temp);
    }

    return (
        <Tile
            borderWidth={10}
            header={
                <div>
                    {tabs.map((tab, i) =>
                    (i === currentTab || (i === 0 && currentTab >= tabs.length) ? (
                        <div
                            key={i}
                            align='center'
                            style={{
                                borderRadius: 2,
                                fontFamily: 'inherit',
                                fontWeight: 400,
                                height: 32,
                                display: 'inline-block',
                                textAlign: 'center',
                                backgroundColor: primaryColor,
                                fontVariant: 'tabular-nums',
                                lineHeight: 1.5715,
                                padding: '6px 15px',
                                marginRight: 10,
                                fontSize: 14
                            }}>
                            {tab.title}
                        </div>
                    ) : (
                        <Button
                            key={i}
                            style={{ marginRight: 10 }}
                            type='ghost'
                            onClick={() => setCurrentTab(i)}>{tab.title}
                        </Button>
                    ))
                    )}
                </div>}>
            <div style={{ height: 'calc(100vh - 556pt)', overflowY: 'scroll' }}>
                {tabs[currentTab] ? tabs[currentTab].contents : tabs[0].contents}
            </div>
        </Tile>
    )
}
