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
                </Blurb>
                
                {frame === 'safety' && (
                    <Blurb highlight={primaryColor}>
                        <h3 style={{color:primaryColor}}>Safety Concerns</h3>
                       
                    </Blurb>
                )}
                {frame === 'performance' && (
                    <Blurb highlight={primaryColor}>
                        <h3 style={{color:primaryColor}}>Robot Performance</h3>
                        
                    </Blurb>
                )}
            </div>
        }
    )
    return tabs
}