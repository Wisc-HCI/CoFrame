import React, {useCallback} from 'react';
import { Space, List, Badge, Switch } from 'antd';

import useReviewStore from '../../stores/ReviewStore';

export const ReviewIssue = ({issueId}) => {

    let issue = useReviewStore(useCallback(state=>state.issues[issueId],[issueId]));
    let completeIssue = useReviewStore(state=>state.completeIssue);

    return (
    <List.Item 
        extra={
            <Space>
                {issue.force && !issue.complete && (
                    <Badge count={'Change Required'}/>
                )}
                <Switch size='small' disabled={issue.force} defaultChecked={issue.complete} onChange={()=>{!issue.force && completeIssue(issueId)}}/>
            </Space>
        }
        style={{
            borderRadius: 3,
            backgroundColor: '#1f1f1f',
            margin: 5, padding: 10
        }}
        >
        <List.Item.Meta title={issue.description} />
    </List.Item>
    )
}