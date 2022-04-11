import React from 'react';
// import { Tooltip } from 'antd';
import { Blurb } from './Blurb';
import { Glossary } from './Glossary';

export function getProcessInfo({frame,primaryColor,focusData,currentIssue,description}) {
    let tabs = [];
    if (currentIssue) {
       
    }; 
    
    tabs.push(
        {
            title:<span>{focusData ? focusData.name : 'Processes'}</span>,
            contents:<div>
                {focusData && (
                    <Blurb highlight="rgb(50,50,50)">
                        <h3>About this Process</h3>
                        {description}
                    </Blurb>
                )}
                <Blurb highlight="rgb(50,50,50)">
                    <h3>What are Processes?</h3>
                    Processes are like recipes that allow various 
                    {' '}
                    <Glossary.Inputs primaryColor={primaryColor} />
                    {' '}
                    to be converted into a set of 
                    {' '}
                    <Glossary.Outputs primaryColor={primaryColor} />.
                    This may need to be accomplished with the assistance of 
                    {' '}
                    <Glossary.Machines primaryColor={primaryColor} />
                    {' '}
                    and 
                    {' '}
                    <Glossary.Tools primaryColor={primaryColor} />.
                    They require a certain amount of time to complete.
                </Blurb>
                
                {frame === 'quality' && (
                    <Blurb highlight={primaryColor}>
                        <h3 style={{color:primaryColor}}>Program Quality</h3>
                        When running a process, if the process requires a {' '} 
                        <Glossary.Machines primaryColor={primaryColor} />
                        , make sure to match that machine in the corresponding process-related 
                        {' '} 
                        <Glossary.Actions primaryColor={primaryColor}/>.

                        Processes take time to execute, so if you start a process, you won’t be able to use its outputs until the process is finished and the time is up.


                       
                    </Blurb>
                )}
                {/* {frame === 'performance' && (
                    <Blurb highlight={primaryColor}>
                        <h3 style={{color:primaryColor}}>Robot Performance</h3>
                        
                    </Blurb>
                )} */}
            </div>
        }
    )
    return tabs
}