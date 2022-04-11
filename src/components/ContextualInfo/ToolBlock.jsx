import React from 'react';
// import { Tooltip } from 'antd';
import { Blurb } from './Blurb';
import { Glossary } from './Glossary';

export function getToolInfo({frame,primaryColor,focusData,currentIssue,description}) {
    let tabs = [];
    if (currentIssue) {
       
    }; 
    
    tabs.push(
        {
            title:<span>{focusData ? focusData.name : 'Tools'}</span>,
            contents:<div>
                {focusData && (
                    <Blurb highlight="rgb(50,50,50)">
                        <h3>About this Tool</h3>
                        {description}
                    </Blurb>
                )}
                <Blurb highlight="rgb(50,50,50)">
                    <h3>What are Tools?</h3>
                    Tools are objects in the scene. Like 
                    {' '}
                    <Glossary.Tools primaryColor={primaryColor}/>
                    {' '}
                    they can be carried around (if light enough), but are not created or consumed by 
                    {' '}
                    <Glossary.Processes primaryColor={primaryColor}/>.

                </Blurb>
                
                {/* {frame === 'safety' && (
                    <Blurb highlight={primaryColor}>
                        <h3 style={{color:primaryColor}}>Safety Concerns</h3>
                       
                    </Blurb>
                )}
                {frame === 'performance' && (
                    <Blurb highlight={primaryColor}>
                        <h3 style={{color:primaryColor}}>Robot Performance</h3>
                        
                    </Blurb>
                )} */}
            </div>
        }
    )
    return tabs
}