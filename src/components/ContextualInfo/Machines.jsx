import React from 'react';
import { Tooltip } from 'antd';
import { Blurb } from './Blurb';

export function getMachineInfo({editorPane,setupTab,frame,primaryColor,focusData,secondaryFocusData,currentIssue}) {
    let tabs = [];
    if (currentIssue) {
        tabs.push(
            {
                title: currentIssue.title, 
                contents:<div>
                    <Blurb highlight={primaryColor}>
                        <h3>{currentIssue.description}</h3>
                        {currentIssue.requiresChanges && (
                            <span>
                                <br></br>
                                <br></br>
                                <i>Since this machine is used in the program, you will need to address this issue in order to simulate the program, since it would prevent the robot from interacting with it.</i>
                            </span>
                        )}
                    </Blurb>
                </div>
            }
        )
    }; 
    tabs.push(
        {
            title:<span>{focusData ? focusData.name : 'Machines'}</span>,
            contents:<div>
                {focusData && (
                    <Blurb highlight="rgb(50,50,50)">
                        <h3>About this Machine</h3>
                        {focusData.description}
                    </Blurb>
                )}
                <Blurb highlight="rgb(50,50,50)">
                    <h3>What are Machines?</h3>
                    Machines are components that are able to perform create, consume, or modify 
                    <Tooltip placement="top" title="A part or piece that has been defined in the setup tab.">
                        <span style={{color:primaryColor}}>
                            {" "}Things
                        </span>
                    </Tooltip> in the program. They define specific 
                    <Tooltip placement="top" title="A geometrical representation of a zone with some variability in either position or rotation">
                        <span style={{color:primaryColor}}>
                            {" "}Regions
                        </span>
                    </Tooltip> that are used for depositing or retrieving these things.
                </Blurb>
                
                {frame === 'safety' && (
                    <Blurb highlight={primaryColor}>
                        <h3 style={{color:primaryColor}}>Safety Concerns</h3>
                        Machines define a collision zone around them, which you can observe by toggling them on and off in the Simulator settings. Trajectories must avoid these zones to avoid collisions. 
                    </Blurb>
                )}
            </div>
        }
    )
    return tabs;
}