import React from 'react';
import {List, Box, Button} from 'grommet';
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
        <Box direction='column' width='100%' flex pad={{top:'xsmall',bottom:'xsmall',left:'xsmall'}}>
            <Box direction='row' gap='xsmall' margin={{bottom:'xsmall'}}>
                {FRAMES.map(frame => (
                    <FrameButton
                        key={frame.key}
                        onClick={() => setFrame(frame.key)}
                        frame={frame.key}
                        active={frameId === frame.key}
                        text={frame.title} />
                ))}
            </Box>
            <Tile
                style={{ height:'100%',backgroundColor:'black'}}
                borderWidth={5}
                internalPaddingWidth={0.1}
                header={
                    <Box direction='row' width='100%' justify='between' align='center'>
                        <h3 style={{margin:'10pt'}}>
                            Review
                        </h3>
                        <Button secondary size='small' round='none' icon={<SyncOutlined/>} onClick={refresh} label='Refresh'></Button>
                    </Box>
                    
                }
            >
                <div style={{height:'calc(100vh - 144pt)',overflowY:'scroll'}}>
                    <List data={FRAMES[frameIdx].sections} border={false} margin='none' pad='none'>
                        {(section, idx) => (
                            <Box key={idx} animation={{ type: 'fadeIn', delay: idx * 100 }} style={{ marginBottom: 5, width: '100%' }}>
                                <ReviewSection sectionId={section} blocked={idx >= blockages[frameIdx]} initialBlocked={idx === blockages[frameIdx]}/>
                            </Box>
                        )}
                    </List>
                </div>
            </Tile>
        </Box>
    );
};