import React from 'react';
import { Tooltip } from 'antd';
import { Blurb } from './Blurb';

export function getSkillInfo({editorPane,setupTab,frame,primaryColor,focusData,secondaryFocusData,currentIssue}) {
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
            title:<span>{focusData ? focusData.name : 'Skill'}</span>,
            contents:<div>
                {focusData && (
                    <Blurb highlight="rgb(50,50,50)">
                        <h3>About this Skill</h3>
                        {focusData.description}
                    </Blurb>
                )}
                <Blurb highlight="rgb(50,50,50)">
                    <h3>What is a Skill?</h3>
                    A Skill is a modular block of the program that can define its own parameters and execute other
                    <Tooltip placement="top" title="The smallest functional activity that a robot can perform. Examples include 'Move Trajectory', 'Grasp', and 'Initialize Machine'.">
                        <span style={{color:primaryColor}}>
                            {" "}Actions
                        </span>
                    </Tooltip> and 
                    <Tooltip placement="top" title="A special action that executes a procedure, that is specified in a Skill block.">
                        <span style={{color:primaryColor}}>
                            {" "}Skill-Calls
                        </span>
                    </Tooltip>.
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
                        </Tooltip> in your skill.
                    </Blurb>
                )}
            </div>
        }
    );
    return tabs;
}