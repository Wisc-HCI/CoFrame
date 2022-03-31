import React, { useState } from 'react';
import { Box, DropButton, Text, Button } from "grommet";
import { FormEdit } from "grommet-icons";
import { eulerFromQuaternion, quaternionFromEuler } from './Geometry';
import useStore from '../../stores/Store';
import { FiX } from 'react-icons/fi';
import Collapse from '../Collapse';
import { NumberInput } from '../NumberInput';


function JointGripperInput({ robotID, isGripper }) {
    let initialStateInfo = [];
    if (isGripper === true) {
        const typeInfo = useStore(state => state.programSpec.objectTypes.gripperType);
         initialStateInfo = useStore(state => {
            state.programData[robotID].properties.initialGripState;
        })

    } else {
        const typeInfo = useStore(state => state.programSpec.objectTypes.robotAgentType);
        initialStateInfo = useStore(state => {
            let list = [];
            for (const [key, value] of Object.entries(state.programData[robotID].properties.initialJointState)) {
                list.push({ key, value });
            }
            console.log(list);
            return list;         
    })
    }
    const primaryColor = useStore(state => state.primaryColor);

    const addFocusItem = useStore(state => state.addFocusItem);

    if (isGripper) {
        return (
            <>
                <Box direction='row' background='#303030' round="xsmall" pad="small" style={{ marginBottom: 5 }} justify='between' wrap={true}>
                    <b style={{ color: 'rgba(255, 255, 255, 0.85)' }} >Initial Gripper State : </b>
                    <Box direction='row' >
                        <div style={{ paddingLeft: "10%" }}>
                            <NumberInput
                                value={initialStateInfo}
                                min={0}
                                max={Infinity}
                            // onChange={(value) => updateItemSimpleProperty(item.id, 'processTime', value)}

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
                    { initialStateInfo.map((io, i) => {

                        return (
                            <Box key={i} round="xsmall" background="rgba(100,100,100,0.3)" direction='row'
                                elevation="none" pad="xsmall" justify='between'
                                hoverIndicator={true}>
                                <b style={{ color: 'rgba(255, 255, 255, 0.85)' }}> {io.key} </b>
                                <div >
                                <NumberInput
                                    value={io.value}
                                    min={0}
                                    max={Infinity}
                                />
                                </div>

                            </Box>
                        )
                    })} 


                </Collapse>

            </>
        );

    }


}
export default (JointGripperInput);
