import React from 'react';
import { Box } from "grommet";

import useStore from '../../stores/Store';
import Collapse from '../Collapse';
import { NumberInput } from '../NumberInput';


function JointGripperInput({ robotID, isGripper }) {
    let initialStateInfo = [];
    let initialState = {};
    let initialStateValue;
    if (isGripper === true) {
       
        initialStateValue = useStore(state =>state.programData[robotID].properties.initialGripState);

    } else {
        initialStateInfo = useStore(state => {
            let list = [];
            for (const [key, value] of Object.entries(state.programData[robotID].properties.initialJointState)) {
                list.push({ key, value });
            }
            return list;
        })
        initialState = useStore(state =>
            state.programData[robotID].properties.initialJointState
        );




    }

    const updateItemSimpleProperty = useStore(state => state.updateItemSimpleProperty);

    if (isGripper) {
        return (
            <>
                <Box direction='row' background='#303030' round="xsmall" pad="small" style={{ marginBottom: 5 }} justify='between' wrap={true}>
                    <b style={{ color: 'rgba(255, 255, 255, 0.85)' }} >Initial Gripper State : </b>
                    <Box direction='row' >
                        <div style={{ paddingLeft: "10%" }}>
                            <NumberInput
                                value={initialStateValue}
                                min={0}
                                max={Infinity}
                                onChange={(value) => updateItemSimpleProperty(robotID, 'initialGripState', value)}
                            />
                        </div>
                    </Box>
                </Box>

            </>);
    } else {
        return (
            <>
                <Collapse
                    openable={true}
                    borderWidth={3}
                    defaultOpen={true}
                    style={{ backgroundColor: '#303030', marginBottom: 5 }}
                    backgroundColor='#202020'
                    header={<Box direction='row' pad="10pt">{isGripper ?
                        <b style={{ color: 'rgba(255, 255, 255, 0.85)' }} >Initial Gripper States : </b> :
                        <b style={{ color: 'rgba(255, 255, 255, 0.85)' }} >Initial Joint States : </b>}{' '}
                    </Box>}
                >
                    {initialStateInfo.map((io, i) => {

                        return (
                            <>
                            <div style ={{"paddingBottom" : "3%"}}>
                            <Box key={i} round="xsmall" background="rgba(100,100,100,0.3)" direction='row'
                                elevation="none" pad="xsmall" justify='between'
                                hoverIndicator={true} > 
                                <b style={{ color: 'rgba(255, 255, 255, 0.85)' }}> {io.key.replace(/_/g,' ')} </b>
                                <div >
                                    <NumberInput
                                        value={io.value}
                                        min={0}
                                        max={Infinity}
                                        onChange={(value) => updateItemSimpleProperty(robotID, { ...initialState, [io.key]: value })}
                                    />
                                </div>
                                
                            </Box>
                            </div>
                            
                            </>
                            
                        )
                    })}


                </Collapse>

            </>
        );

    }


}
export default (JointGripperInput);
