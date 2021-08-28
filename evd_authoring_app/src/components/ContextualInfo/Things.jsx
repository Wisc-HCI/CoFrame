import React from 'react';
import { Tooltip } from 'antd';
import { Blurb } from './Blurb';

export function getThingInfo({editorPane,setupTab,frame,primaryColor,focusData,secondaryFocusData,currentIssue}) {
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
                                <i>Since this thing is used in the program, you will need to address this issue in order to simulate the program, since it would prevent the robot from performing motions that use it.</i>
                            </span>
                        )}
                    </Blurb>
                </div>
            }
        )
    }
    tabs.push(
        {
            title:<span>{focusData ? focusData.name : 'Things'}</span>,
            contents:<div>
                {focusData && (
                    <Blurb highlight="rgb(50,50,50)">
                        <h3>About this Thing</h3>
                        {focusData.description}
                    </Blurb>
                )}
                <Blurb highlight="rgb(50,50,50)">
                    <h3>What are Things?</h3>
                    Things are the parts that are created, consumed, or modified through the course of the program through the use of
                    <Tooltip placement="top" title="A fixed tool in the environment that is capable of modifying Things according to a set of known recipes.">
                        <span style={{color:primaryColor}}>
                            {" "}Machines
                        </span>
                    </Tooltip>, and moved around by the robot with 
                    <Tooltip placement="top" title="A action that can be added and customized in the Program Editor. To transport Things, you must first Grasp them.">
                        <span style={{color:primaryColor}}>
                            {" "}Move Trajectory Primitives
                        </span>
                    </Tooltip>.
                </Blurb>
                
                {frame === 'safety' && (
                    <Blurb highlight={primaryColor}>
                        <h3 style={{color:primaryColor}}>Safety Concerns</h3>
                        One of the components of a safe robot trajectory is that if the robot is transporting a Thing, that Thing is considered safe. Unsafe things must be handled by the human.
                    </Blurb>
                )}
                {frame === 'performance' && (
                    <Blurb highlight={primaryColor}>
                        <h3 style={{color:primaryColor}}>Robot Performance</h3>
                        In order to have correct robot movement, the robot's payload must be within the limits specified by the robot. This robot has a payload of 3kg, so any Thing greater than this weight cannot be carried in a 
                        <Tooltip placement="top" title="A action that can be added and customized in the Program Editor. To transport Things, you must first Grasp them.">
                            <span style={{color:primaryColor}}>
                                {" "}Move Trajectory Primitive
                            </span>
                        </Tooltip>.
                    </Blurb>
                )}
                {frame === 'business' && (
                    <Blurb highlight={primaryColor}>
                        <h3 style={{color:primaryColor}}>Business Objectives</h3>
                        For measures such as Return on Investment (ROI), be mindful of how your changes to the program impact the number of Things that can be produced in a given amount of time. 
                    </Blurb>
                )}
            </div>
        }
    );
    return tabs;
}