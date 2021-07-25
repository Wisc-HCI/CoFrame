import React, {useCallback} from 'react';

import { List, Switch, Collapse, Row, Space } from 'antd';
import { CaretRightOutlined } from '@ant-design/icons';

import { ReviewIssue } from './ReviewIssue';
import useGuiStore from '../../stores/GuiStore';
import useReviewStore from '../../stores/ReviewStore';

import { FrameButton } from '../FrameButton';

import frameStyles from '../../frameStyles';

import './custom.css'

const isComplete = (state, sectionId) => (state.sections[sectionId].issues.map(issueId=>state.issues[issueId]).filter(issue=>!issue.complete).length === 0);

const FRAMES = [
    { key: 'safety', title: 'Safety Concerns', sections: ['endEffectorPoses', 'thingMovement', 'pinchPoints', 'collisions', 'occupancy'] },
    { key: 'quality', title: 'Program Quality', sections: ['missingBlocks', 'missingParameters', 'unusedSkills', 'unusedFeatures', 'emptyBlocks'] },
    { key: 'performance', title: 'Robot Performance', sections: ['reachability', 'jointSpeed', 'endEffectorSpeed', 'payload', 'spaceUsage'] },
    { key: 'business', title: 'Business Objectives', sections: ['cycleTime', 'idleTime', 'returnOnInvestment'] },
]
const sectionFrame = (sectionId) => FRAMES.filter(filter=>filter.sections.indexOf(sectionId)>-1)[0].key

export function ReviewSection({sectionId, blocked, initialBlocked}) {
  
  const [frame, setFrame] = useGuiStore(state=>([state.frame,state.setFrame]));
  const [name, issueIds, complete, dependencies, depNames, depFrames] = useReviewStore(useCallback(state=>{
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
    <List.Item style={{margin:0,padding:0}}>
        <Space direction='vertical' style={{width:'100%'}}>
            {dependencies.map((dep,i)=>(
                <Row 
                    key={dep} 
                    align='middle'
                    justify='space-between'
                    style={{
                        padding:10,
                        marginBottom:0,marginTop:10,marginLeft:5,marginRight:5,
                        borderRadius:4,
                        border: `1px solid ${frameStyles.colors[depFrames[i]]}`,
                        backgroundColor:`${frameStyles.colors[depFrames[i]]}44`}}>
                        <span style={{marginRight:20}}>Resolve <b>{depNames[i]}</b> before continuing</span>
                        {(frame !== depFrames[i]) && (<FrameButton frame={depFrames[i]} active={false} text={`Go to ${FRAMES[i].title}`} onClick={()=>setFrame(depFrames[i])}/>)}
                </Row>
            ))}
            <Collapse
                collapsible={!blocked ? null : 'disabled'} 
                expandIcon={({ isActive }) => <CaretRightOutlined style={{color:'white'}} rotate={isActive ? 90 : 0} />}
                style={{
                    flex:1, 
                    borderRadius: 3,
                    opacity: !blocked ? 1 : 0.5,
                    backgroundColor: '#1f1f1f',
                    marginTop: 5, marginLeft: 5, marginRight: 5, marginBottom:0}}
                expandIconPosition='right'
            >
                <Collapse.Panel style={{margin:0}} header={<span style={{color:'white'}}>{name}</span>} key="1" extra={<Switch checkedChildren='Done!' checked={complete}/>}>
                    <List
                        split={false}
                        dataSource={issueIds}
                        renderItem={(issue)=>(
                            <ReviewIssue key={issue} issueId={issue}/>
                        )}
                    />
                </Collapse.Panel>
            </Collapse>
        </Space>
    </List.Item>
  );
};