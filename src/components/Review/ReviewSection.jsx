import React, {useCallback} from 'react';

// import { List, Switch, Row, Space } from 'antd';
// import { CaretRightOutlined } from '@ant-design/icons';
import { Box, List } from 'grommet';
import { Toggle } from '../Toggle';

import { ReviewIssue } from './ReviewIssue';
import useStore from '../../stores/Store';

import { FrameButton } from '../FrameButton';

import frameStyles from '../../frameStyles';

import Collapse from '../Collapse';

// import './custom.css'

const isComplete = (state, sectionId) => (state.sections[sectionId].issues.map(issueId=>state.issues[issueId]).filter(issue=>!issue.complete).length === 0);

const FRAMES = [
    { key: 'safety', title: 'Safety Concerns', sections: ['endEffectorPoses', 'thingMovement', 'pinchPoints', 'collisions', 'occupancy'] },
    { key: 'quality', title: 'Program Quality', sections: ['missingBlocks', 'missingParameters', 'machineLogic', 'unusedSkills', 'unusedFeatures', 'emptyBlocks'] },
    { key: 'performance', title: 'Robot Performance', sections: ['reachability', 'jointSpeed', 'endEffectorSpeed', 'payload', 'spaceUsage'] },
    { key: 'business', title: 'Business Objectives', sections: ['cycleTime', 'idleTime', 'returnOnInvestment'] },
]

const NAMES = {
    safety: 'Safety Concerns', 
    quality: 'Program Quality', 
    performance: 'Robot Performance', 
    business: 'Business Objectives'
};

const sectionFrame = (sectionId) => FRAMES.filter(filter=>filter.sections.indexOf(sectionId)>-1)[0].key

export function ReviewSection({sectionId, blocked, initialBlocked}) {
  
  const [frame, setFrame, primaryColor] = useStore(state=>([state.frame,state.setFrame,state.primaryColor]));
  const [name, issueIds, complete, dependencies, depNames, depFrames] = useStore(useCallback(state=>{
    const name = state.sections[sectionId].name;
    const issueIds = state.sections[sectionId].issues;
    const complete = isComplete(state,sectionId);
    const dependencies = initialBlocked ? state.sections[sectionId].dependencies.filter(dep=>!isComplete(state,dep)) : [];
    const depNames = dependencies.map(dep=>state.sections[dep].name);
    const depFrames = dependencies.map(dep=>sectionFrame(dep));
    return [name, issueIds, complete, dependencies, depNames, depFrames];
  },
  [sectionId, initialBlocked]))

  return (
    <Box style={{margin:0,padding:0}}>
        <Box direction='column' width='100%'>
            {dependencies.map((dep,i)=>(
                <Box 
                    key={dep} 
                    direction='row'
                    align='middle'
                    justify='between'
                    style={{
                        padding:10,
                        marginBottom:0,marginTop:10,marginLeft:5,marginRight:5,
                        borderRadius:4,
                        border: `1px solid ${frameStyles.colors[depFrames[i]]}`,
                        backgroundColor:`${frameStyles.colors[depFrames[i]]}44`}}>
                        <span style={{marginRight:20}}>Resolve <b>{depNames[i]}</b> before continuing</span>
                        {(frame !== depFrames[i]) && (<FrameButton frame={depFrames[i]} active={false} text={`Go to ${NAMES[sectionFrame(dependencies[i])]}`} onClick={()=>setFrame(depFrames[i])}/>)}
                </Box>
            ))}
            
            <Collapse
                openable={!blocked}
                borderWidth={3}
                header={<Box direction='row' pad="10pt">{name}</Box>}
                style={{marginBottom:5}}
                extra={
                <Toggle 
                    selectedText="Done!"
                    deselectedText=""
                    backgroundColor={primaryColor}
                    selected={complete} 
                    disabled
                />}
            >
                {issueIds.length > 0 ? <List
                    split={false}
                    data={issueIds}
                >
                    {(issue)=>(
                        <ReviewIssue key={issue} issueId={issue}/>
                    )}
                </List> : null}
            </Collapse>
        </Box>
    </Box>
  );
};