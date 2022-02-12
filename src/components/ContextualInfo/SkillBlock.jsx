import React from 'react';
import { Blurb } from './Blurb';
import { Glossary } from './Glossary';

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
                    <Glossary.Actions primaryColor={primaryColor}/> and 
                    <Glossary.SkillCalls primaryColor={primaryColor}/>.
                </Blurb>
                
                {frame === 'quality' && (
                    <Blurb highlight={primaryColor}>
                        <h3 style={{color:primaryColor}}>Program Quality</h3>
                        Make sure to fully parameterize all 
                        <Glossary.Actions primaryColor={primaryColor}/> and 
                        <Glossary.SkillCalls primaryColor={primaryColor}/> in your skill.
                    </Blurb>
                )}
            </div>
        }
    );
    return tabs;
}