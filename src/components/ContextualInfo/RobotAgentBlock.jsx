import React from 'react';
// import { Tooltip } from 'antd';
import { Blurb } from './Blurb';
import { Glossary } from './Glossary';

export function getRobotAgentInfo({frame,primaryColor,focusData,currentIssue,description}) {
    let tabs = [];
    if (currentIssue) {
       
    }; 
    
    tabs.push(
        {
            title:<span>{focusData ? focusData.name : 'Robot Agent:'}</span>,
            contents:<div>
                {focusData && (
                    <Blurb highlight="rgb(50,50,50)">
                        <h3>About this Robot Agent</h3>
                        {description}
                    </Blurb>
                )}
                <Blurb highlight="rgb(50,50,50)">
                    <h3>What are Robot Agents?</h3>
                    Robot Agents are the physical robots/cobots that cooperate with Machines to execute the given 
                    {' '}
                    <Glossary.RobotAgents primaryColor={primaryColor}/>.
                </Blurb>
                
                {frame === 'safety' && (
                    <Blurb highlight={primaryColor}>
                        <h3 style={{color:primaryColor}}>Safety Concerns</h3>
                        Be mindful of the time the robot spends in human-designated areas, and how quickly the robot moves, since this can negatively affect safety measures.
                    </Blurb>
                )}
                {frame === 'performance' && (
                    <Blurb highlight={primaryColor}>
                        <h3 style={{color:primaryColor}}>Robot Performance</h3>
                        Robots drive their segments (links) by altering their joint values. The faster these joints move, the faster the robot moves from place to place. Each robot also has a maximum payload, or amount that it can carry.
                    </Blurb>
                )}
            </div>
        }
    )
    return tabs
}