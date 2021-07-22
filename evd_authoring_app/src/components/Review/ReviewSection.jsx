import React, {useCallback} from 'react';

import { List, Switch, Collapse, Alert } from 'antd';
import { CaretRightOutlined } from '@ant-design/icons';

import { ReviewIssue } from './ReviewIssue';
import useReviewStore from '../../stores/ReviewStore';

import './custom.css'

const isComplete = (state, sectionId) => (state.sections[sectionId].issues.map(issueId=>state.issues[issueId]).filter(issue=>!issue.complete).length === 0);

export function ReviewSection({sectionId, blocked, initialBlocked}) {

  const [name, issueIds, complete, dependencies] = useReviewStore(useCallback(state=>{
    const name = state.sections[sectionId].name;
    const issueIds = state.sections[sectionId].issues;
    const complete = isComplete(state,sectionId);
    const dependencies = initialBlocked ? state.sections[sectionId].dependencies.filter(dep=>isComplete(state,dep)) : [];
    return [name, issueIds, complete, dependencies];
  },
  [sectionId, initialBlocked]))

  //   console.log(`${sectionId} ${blocked} ${initialBlocked}`)

  return (
    <List.Item style={{margin:0,padding:0}}>
        {dependencies.map(dep=>(
            <Alert key={dep}/>
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
    </List.Item>
  );
};