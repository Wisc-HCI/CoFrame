import React, {useCallback} from 'react';
import { Space, List, Avatar, Switch, Button } from 'antd';
import { EllipsisOutlined, StopOutlined, ExclamationOutlined } from '@ant-design/icons';
import useStore from '../../stores/Store';

export const ReviewIssue = ({issueId}) => {

    const issue = useStore(useCallback(state=>state.issues[issueId],[issueId]));
    const [setIssueCompletion,setFocusItem,primaryColor,secondaryFocusItem,setSecondaryFocusItem] = useStore(state=>([state.setIssueCompletion,state.setFocusItem,state.primaryColor, state.secondaryFocusItem, state.setSecondaryFocusItem]));

    return (
    <List.Item 
        extra={
            <Space direction='vertical' align='center'>
                {!issue.requiresChanges && (
                    <Switch size='small' disabled={issue.requiresChanges} defaultChecked={issue.complete} onChange={()=>{!issue.requiresChanges && setIssueCompletion(issueId,!issue.complete)}}/>
                )}
                {issue.focus && <Button onClick={()=>{setFocusItem(issue.focus.type,issue.focus.uuid);setSecondaryFocusItem('issue',issueId)}} icon={<EllipsisOutlined/>}/>}
            </Space>
        }
        style={{
            borderRadius: 3,
            boxShadow: secondaryFocusItem.uuid === issueId ? `inset 0pt 0pt 1pt 1pt ${primaryColor}` : null,
            backgroundColor: secondaryFocusItem.uuid === issueId ? `${primaryColor}22` : '#1f1f1f',
            margin: 5, padding: 10
        }}
        >
        <List.Item.Meta 
            style={{marginRight:10}}
            avatar={<Avatar style={{backgroundColor: issue.requiresChanges ? '#a61d24' : 'grey'}} icon={issue.requiresChanges ? <StopOutlined style={{fontSize:20}}/> : <ExclamationOutlined style={{fontSize:20}}/>}/>} 
            title={issue.title} 
            description={issue.requiresChanges?'Please reconfigure the item to remove this error':'Check to dismiss'}/>
    </List.Item>
    )
}