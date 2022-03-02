import React from 'react';
import { Glossary } from './Glossary';
import { Blurb } from './Blurb';

export function getPrimitiveInfo({editorPane,setupTab,frame,primaryColor,focusData,secondaryFocusData,currentIssue}) {
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
                        <h3>About this Action</h3>
                        {focusData.description}
                    </Blurb>
                )}
                <Blurb highlight="rgb(50,50,50)">
                    <h3>What is an Action?</h3>
                    Actions are the smallest functional activity that a robot can perform in the 
                    <Glossary.Program primaryColor={primaryColor}/>. 
                    Examples include 'Move Trajectory', 'Grasp', and 'Initialize Machine'.
                </Blurb>
                
                {frame === 'quality' && (
                    <Blurb highlight={primaryColor}>
                        <h3 style={{color:primaryColor}}>Program Quality</h3>
                        Make sure to fully parameterize this Action fully by dragging in blocks from the sidebar, or from any parent 
                        <Glossary.Skills primaryColor={primaryColor}/>
                        for inspiration.
                    </Blurb>
                )}
            </div>
        }
    );
    return tabs;
}