import React from 'react';
import { Blurb } from './Blurb';
import { Glossary } from './Glossary';

export function getMachineInfo({frame,primaryColor,focusData,currentIssue,description}) {
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
                        {description}
                    </Blurb>
                )}
                <Blurb highlight="rgb(50,50,50)">
                    <h3>What are Machines?</h3>
                    Machines are components that are able to perform create, consume, or modify
                    {' '} 
                    <Glossary.Things primaryColor={primaryColor}/>
                    {' '}
                    in the program. They define specific 
                    {' '}
                    <Glossary.Regions primaryColor={primaryColor}/>
                    {' '}
                    that are used for depositing or retrieving these things.
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