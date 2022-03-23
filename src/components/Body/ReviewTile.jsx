import React from 'react';
import { List, Box, Button } from 'grommet';
import Tile from '../Tile';
import { FrameButton } from '../FrameButton';
import { FiRefreshCw } from "react-icons/fi";
// import frameStyles from '../../frameStyles';
import useStore from '../../stores/Store';
import shallow from 'zustand/shallow';
import { ReviewSection } from '../Review/ReviewSection';
// import { animated } from '@react-spring/web';
import useMeasure from 'react-use-measure';

const isComplete = (state, sectionId) => (state.sections[sectionId].issues.map(issueId => state.issues[issueId]).filter(issue => !issue.complete).length === 0);
const isBlocked = (state, sectionId) => (state.sections[sectionId].dependencies.filter(dep => !isComplete(state, dep)).length > 0)

const FRAMES = [
    { key: 'safety', title: 'Safety Concerns', sections: ['endEffectorPoses', 'thingMovement', 'collisions', 'pinchPoints', 'occupancy'] },
    { key: 'quality', title: 'Program Quality', sections: ['missingBlocks', 'missingParameters', 'machineLogic', 'unusedSkills', 'unusedFeatures', 'emptyBlocks'] },
    { key: 'performance', title: 'Robot Performance', sections: ['reachability', 'jointSpeed', 'endEffectorSpeed', 'payload', 'spaceUsage'] },
    { key: 'business', title: 'Business Objectives', sections: ['cycleTime', 'idleTime', 'returnOnInvestment'] },
]

export const ReviewTile = (_) => {
    const [frameId, setFrame, refresh, blockages] = useStore(state => ([
        state.frame,
        state.setFrame,
        state.refresh,
        FRAMES.map(frameInfo => Math.min(...frameInfo.sections.map((sectionId, idx) => isBlocked(state, sectionId) ? idx : 100)))
    ]), shallow);

    const frameIdx = FRAMES.map(frame => frame.key).indexOf(frameId);

    const [ ref, bounds ] = useMeasure();

    return (
        <Box ref={ref} flex direction='column'>
            <Tile
            style={{ height:bounds.height-10, backgroundColor: 'black' }}
            borderWidth={5}
            internalPaddingWidth={0.1}
            backgroundColor='black'
            header={
                <Box direction='column'>
                    <Box direction='row' gap='xsmall' margin={{ bottom: 'xsmall' }}>
                        {FRAMES.map(frame => (
                            <FrameButton
                                key={frame.key}
                                onClick={() => setFrame(frame.key)}
                                frame={frame.key}
                                active={frameId === frame.key}
                                text={frame.title} />
                        ))}
                    </Box>
                    <Box direction='row' width='100%' justify='between' align='center'>
                        <h3 style={{ margin: '10pt' }}>
                            Review
                        </h3>
                        <Button secondary size='small' round='none' icon={<FiRefreshCw/>} onClick={refresh} label='Refresh'></Button>
                    </Box>
                </Box>


            }
        >
            <div style={{ height: bounds.height-125, overflowY: 'scroll' }}>
                <List data={FRAMES[frameIdx].sections} border={false} margin='none' pad='none'>
                    {(section, idx) => (
                        <Box key={idx} animation={{ type: 'fadeIn', delay: idx * 100 }} style={{ marginBottom: 5, width: '100%' }}>
                            <ReviewSection sectionId={section} blocked={idx >= blockages[frameIdx]} initialBlocked={idx === blockages[frameIdx]} />
                        </Box>
                    )}
                </List>
            </div>
        </Tile>
        </Box>
        
    );
};