import React from 'react';
import { Blurb } from './Blurb';
import { Glossary } from './Glossary';

export function getTrajectoryInfo({frame,primaryColor,focusData,currentIssue,description}) {
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
                        {description}
                    </Blurb>
                )}
                <Blurb highlight="rgb(50,50,50)">
                    <h3>What is the Program?</h3>
                    The Program is the main sequence of 
                    <Glossary.Actions primaryColor={primaryColor}/> and 
                    <Glossary.SkillCalls primaryColor={primaryColor}/> that are executed by the robot.
                </Blurb>
                
                {frame === 'quality' && (
                    <Blurb highlight={primaryColor}>
                        <h3 style={{color:primaryColor}}>Program Quality</h3>
                        Make sure to fully parameterize all 
                        <Glossary.Actions primaryColor={primaryColor}/> and 
                        <Glossary.SkillCalls primaryColor={primaryColor}/> in your program. Also, make sure to check all the existing 
                        <Glossary.Skills primaryColor={primaryColor}/> for inspiration.
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