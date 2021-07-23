import React from 'react';
import { List, Card, Space, Button } from 'antd';
import { SyncOutlined } from '@ant-design/icons';

import { FrameButton } from '../FrameButton';

import frameStyles from '../../frameStyles';
import useGuiStore from '../../stores/GuiStore';
import useReviewStore from '../../stores/ReviewStore';

import { ReviewSection } from '../Review/ReviewSection';

const isComplete = (state, sectionId) => (state.sections[sectionId].issues.map(issueId=>state.issues[issueId]).filter(issue=>!issue.complete).length === 0);

const FRAMES = [
    { key: 'safety', title: 'Safety Concerns', sections: ['endEffectorPoses', 'thingMovement', 'pinchPoints', 'collisions', 'occupancy'] },
    { key: 'quality', title: 'Program Quality', sections: ['missingBlocks', 'missingParameters', 'unusedSkills', 'unusedFeatures', 'emptyBlocks'] },
    { key: 'performance', title: 'Robot Performance', sections: ['reachability', 'jointSpeed', 'endEffectorSpeed', 'payload', 'spaceUsage'] },
    { key: 'business', title: 'Business Objectives', sections: ['cycleTime', 'idleTime', 'returnOnInvestment'] },
]

export const ReviewTile = (_) => {
    const { frameId, setFrame } = useGuiStore(state => ({ frameId: state.frame, setFrame: state.setFrame }));
    const refresh = useReviewStore(state => state.refresh);
    const blockages = useReviewStore(state => FRAMES.map(frameInfo=>Math.min(...frameInfo.sections.map((sectionId,idx)=>!isComplete(state,sectionId)?idx:100))));
    const frameIdx = FRAMES.map(frame=>frame.key).indexOf(frameId);

    console.log(blockages);

    return (
        <div style={{ height: '100%', paddingLeft: 10, paddingRight: 10, paddingBottom: 10, display: 'flex', flexDirection: 'column' }}>
            <Space style={{ flexDirection: 'row', display: 'flex', justifyContent: 'space-between', margin: 5 }}>
                {FRAMES.map(frame => (
                    <FrameButton
                        key={frame.key}
                        onClick={() => setFrame(frame.key)}
                        frame={frame.key}
                        active={frameId === frame.key}
                        text={frame.title} />
                ))}
            </Space>
            <Card
                extra={<Button icon={<SyncOutlined />} onClick={refresh}>Refresh</Button>}
                style={{ flex: 1 }}
                bodyStyle={{ padding: 0, height:'calc(100vh - 165pt)',overflow:'auto'}}
                title="Review"
            >
                <div style={{ height: '100%', borderWidth: 5, borderColor: frameStyles.colors[frameId] }}>
                    <List
                        split={false}
                        dataSource={FRAMES[frameIdx].sections}
                        renderItem={(section,idx) => (
                            <ReviewSection sectionId={section} blocked={idx >= blockages[frameIdx]} initialBlocked={idx === blockages[frameIdx]}/>
                        )}
                    />
                </div>
            </Card>
        </div>
    );
};