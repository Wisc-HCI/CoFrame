import React from 'react';
import { Tooltip } from 'antd';
import { Blurb } from './Blurb';

export function getTrajectoryInfo({editorPane,setupTab,frame,primaryColor,focusData,secondaryFocusData,currentIssue}) {
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
                                <i>Since this issue is required, you will need to address this issue in order to simulate.</i>
                            </span>
                        )}
                    </Blurb>
                </div>
            }
        )
    }
    tabs.push(
        {
            title:<span>{focusData ? focusData.name : 'Program'}</span>,
            contents:<div>
                {focusData && (
                    <Blurb highlight="rgb(50,50,50)">
                        <h3>About the Program</h3>
                        {focusData.description}
                    </Blurb>
                )}
                <Blurb highlight="rgb(50,50,50)">
                    <h3>What is the Program?</h3>
                    The Program is the main sequence of 
                    <Tooltip placement="top" title="The smallest functional activity that a robot can perform. Examples include 'Move Trajectory', 'Grasp', and 'Initialize Machine'.">
                        <span style={{color:primaryColor}}>
                            {" "}Actions
                        </span>
                    </Tooltip> and 
                    <Tooltip placement="top" title="A special action that executes a procedure, that is specified in a Skill block.">
                        <span style={{color:primaryColor}}>
                            {" "}Skill-Calls
                        </span>
                    </Tooltip> that are executed by the robot.
                </Blurb>
                
                {frame === 'quality' && (
                    <Blurb highlight={primaryColor}>
                        <h3 style={{color:primaryColor}}>Program Quality</h3>
                        Make sure to fully parameterize all 
                        <Tooltip placement="top" title="The smallest functional activity that a robot can perform. Examples include 'Move Trajectory', 'Grasp', and 'Initialize Machine'.">
                        <span style={{color:primaryColor}}>
                            {" "}Actions
                        </span>
                        </Tooltip> and 
                        <Tooltip placement="top" title="A special action that executes a procedure, that is specified in a Skill block.">
                            <span style={{color:primaryColor}}>
                                {" "}Skill-Calls
                            </span>
                        </Tooltip> in your program. Also, make sure to check all the existing 
                        <Tooltip placement="top" title="A modular collection of actions that can be executed by using a Skill-Call">
                            <span style={{color:primaryColor}}>
                                {" "}Skills
                            </span>
                        </Tooltip> for inspiration.
                    </Blurb>
                )}
                {frame === 'business' && (
                    <Blurb highlight={primaryColor}>
                        <h3 style={{color:primaryColor}}>Business Objectives</h3>
                        The Program can be executed on a loop, so shorter program durations lower the cycle time, and usually result in an increase in Return on Investment (ROI).
                    </Blurb>
                )}
            </div>
        }
    );
    return tabs;
}