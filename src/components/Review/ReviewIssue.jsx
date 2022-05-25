import React, {useCallback} from 'react';
// import { Space, List, Avatar, Switch, Button } from 'antd';
import { Box, Avatar, Button, Text } from 'grommet';
// import { EllipsisOutlined, StopOutlined, ExclamationOutlined } from '@ant-design/icons';
import { FiSlash, FiAlertCircle, FiMoreHorizontal } from 'react-icons/fi';
import useStore from '../../stores/Store';
import { Switch } from '../Switch';

export const ReviewIssue = ({issueId}) => {

    const issue = useStore(useCallback(state=>state.issues[issueId],[issueId]));
    const [
        setIssueCompletion,
        primaryColor,
        addFocusItem,
        focus
    ] = useStore(state=>([
        state.setIssueCompletion,
        state.primaryColor, 
        state.addFocusItem,
        state.focus
    ]));

    const Icon = issue.requiresChanges ? FiSlash : FiAlertCircle; 

    return (
    <Button onClick={()=>{addFocusItem(issue.focus.id, true);addFocusItem(issueId, true)}}>
        <Box 
            direction='row'
            flex
            round='small'
            align='center'
            background={focus.includes(issueId) ? `${primaryColor}22` : '#1f1f1f'}
            style={{
                boxShadow: focus.includes(issueId) ? `inset 0pt 0pt 1pt 1pt ${primaryColor}` : null,
                padding: "5px",
                paddingTop: "10px",
                paddingBottom: "10px",
            }}
            >
                <Avatar background={issue.requiresChanges ? '#a61d24' : 'grey'}>
                    <Icon style={{color:"white"}} />
                </Avatar>
                <Box direction='column' flex>
                    <Text>{issue.title}</Text>
                    <Text size="small">{issue.requiresChanges?'Please reconfigure the item to remove this error':'Check to dismiss'}</Text>
                </Box>
                {/* <Box direction='column' align='center'> */}
                    {!issue.requiresChanges && (
                        <Switch label="Done" disabled={issue.requiresChanges} value={issue.complete} onCheckedChange={()=>{!issue.requiresChanges && setIssueCompletion(issueId,!issue.complete)}} highlightColor={primaryColor}/>
                    )}
                    {/* {issue.focus && <Button onClick={()=>{addFocusItem(issue.focus.id, true);addFocusItem(issueId, true)}} icon={<FiMoreHorizontal/>}/>} */}
                {/* </Box> */}
        </Box>
    </Button>
    )
}