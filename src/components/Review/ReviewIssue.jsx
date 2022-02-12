import React, {useCallback} from 'react';
// import { Space, List, Avatar, Switch, Button } from 'antd';
import { Box, Avatar, Button } from 'grommet';
import { Toggle } from '../Toggle';
// import { EllipsisOutlined, StopOutlined, ExclamationOutlined } from '@ant-design/icons';
import { FiSlash, FiAlertCircle, FiMoreHorizontal } from 'react-icons/fi';
import useStore from '../../stores/Store';

export const ReviewIssue = ({issueId}) => {

    const issue = useStore(useCallback(state=>state.issues[issueId],[issueId]));
    const [
        setIssueCompletion,
        setFocusItem,
        primaryColor,
        secondaryFocusItem,
        setSecondaryFocusItem
    ] = useStore(state=>([
        state.setIssueCompletion,
        state.setFocusItem,
        state.primaryColor, 
        state.secondaryFocusItem, 
        state.setSecondaryFocusItem
    ]));

    const Icon = issue.requiresChanges ? FiSlash : FiAlertCircle; 

    return (
    <Box 
        direction='row'
        flex
        round='small'
        margin='5pt'
        pad='10pt'
        background={secondaryFocusItem.uuid === issueId ? `${primaryColor}22` : '#1f1f1f'}
        style={{
            boxShadow: secondaryFocusItem.uuid === issueId ? `inset 0pt 0pt 1pt 1pt ${primaryColor}` : null
        }}
        >
            <Avatar background={issue.requiresChanges ? '#a61d24' : 'grey'}>
                <Icon style={{color:"white"}} />
            </Avatar>
            <Box direction='column' flex>
                <Text>{issue.title}</Text>
                <Text>{issue.requiresChanges?'Please reconfigure the item to remove this error':'Check to dismiss'}</Text>
            </Box>
            <Box direction='column' align='center'>
                {!issue.requiresChanges && (
                    <Toggle size='small' disabled={issue.requiresChanges} selected={issue.complete} onClick={()=>{!issue.requiresChanges && setIssueCompletion(issueId,!issue.complete)}} backgroundColor={primaryColor}/>
                )}
                {issue.focus && <Button onClick={()=>{setFocusItem(issue.focus.type,issue.focus.uuid);setSecondaryFocusItem('issue',issueId)}} icon={<FiMoreHorizontal/>}/>}
            </Box>
    </Box>
    )
}