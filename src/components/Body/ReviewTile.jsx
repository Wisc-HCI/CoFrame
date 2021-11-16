import React from 'react';
import { List, Row, Space, Button } from 'antd';
import { SyncOutlined } from '@ant-design/icons';
import Tile from '../Tile';
import { FrameButton } from '../FrameButton';

import frameStyles from '../../frameStyles';
import useStore from '../../stores/Store';
import shallow from 'zustand/shallow';
import { ReviewSection } from '../Review/ReviewSection';
import { animated } from '@react-spring/web';

const isComplete = (state, sectionId) => (state.sections[sectionId].issues.map(issueId=>state.issues[issueId]).filter(issue=>!issue.complete).length === 0);
const isBlocked = (state, sectionId) => (state.sections[sectionId].dependencies.filter(dep=>!isComplete(state,dep)).length > 0)

const FRAMES = [
    { key: 'safety', title: 'Safety Concerns', sections: ['endEffectorPoses', 'thingMovement', 'collisions', 'pinchPoints', 'occupancy'] },
    { key: 'quality', title: 'Program Quality', sections: ['missingBlocks', 'missingParameters', 'machineLogic', 'unusedSkills', 'unusedFeatures', 'emptyBlocks'] },
    { key: 'performance', title: 'Robot Performance', sections: ['reachability', 'jointSpeed', 'endEffectorSpeed', 'payload', 'spaceUsage'] },
    { key: 'business', title: 'Business Objectives', sections: ['cycleTime', 'idleTime', 'returnOnInvestment'] },
]

export const ReviewTile = (_) => {
    const [frameId, setFrame, refresh, blockages ] = useStore(state => ([
        state.frame, 
        state.setFrame,
        state.refresh,
        FRAMES.map(frameInfo=>Math.min(...frameInfo.sections.map((sectionId,idx)=>isBlocked(state,sectionId)?idx:100)))
    ]), shallow);
    
    const frameIdx = FRAMES.map(frame=>frame.key).indexOf(frameId);

    return (
        <div style={{ height: '100%', paddingLeft: 10, paddingRight: 10, paddingBottom: 10, display: 'flex', flexDirection: 'column' }}>
            <Space style={{ flexDirection: 'row', display: 'flex', justifyContent: 'space-between', marginTop: 5, marginBottom: 5 }}>
                {FRAMES.map(frame => (
                    <FrameButton
                        key={frame.key}
                        onClick={() => setFrame(frame.key)}
                        frame={frame.key}
                        active={frameId === frame.key}
                        text={frame.title} />
                ))}
            </Space>
            <Tile
                style={{ height:'calc(100vh - 100pt)',backgroundColor:'black'}}
                borderWidth={5}
                internalPaddingWidth={0.1}
                header={
                    <Row style={{width:'100%',paddingRight:10}} justify='space-between' align='middle'>
                        <h3 style={{margin:'10pt'}}>
                            Review
                        </h3>
                        <Button icon={<SyncOutlined/>} onClick={refresh}>Refresh</Button>
                    </Row>
                    
                }
            >
                <div style={{height:'calc(100vh - 150pt)',overflowY:'scroll'}}>
                    <List
                        split={false}
                        dataSource={FRAMES[frameIdx].sections}
                        renderItem={(section,idx) => (
                            <ReviewSection sectionId={section} blocked={idx >= blockages[frameIdx]} initialBlocked={idx === blockages[frameIdx]}/>
                        )}
                    />
                </div>
            </Tile>
        </div>
    );
};