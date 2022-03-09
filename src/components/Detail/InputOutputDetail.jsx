import React from 'react';
import useStore from '../../stores/Store';
import { Text, Box } from 'grommet';
import OrientationInput from './OrientationInput';
import PositionInput from './PositionInput';
export const ProcessIOPositionRotationDetail = ({ inputOutputId }) => {
    const position = useStore(state => {
        return state.programData[inputOutputId].properties.position;
    });

    const rotation = useStore(state => {
        return state.programData[inputOutputId].properties.rotation;
    });

    return (
        <>
            <Box
                round="xsmall"
                background="#303030"
                pad="small" >
                <Box direction='column'>
                    <b style={{ color: 'rgba(255, 255, 255, 0.85)', paddingBottom: '2%' }} >Placement : </b>
                    <div style={{ display: 'flex', flexDirection: 'column'}}>
                        <div style={{paddingBottom: "4%"}}>
                        <Box round="xsmall"
                            background="black"
                            pad="small"
                            width= "100%"
                            >
                            <PositionInput position={position} />
                        </Box>
                        </div>
                        

                        <Box round="xsmall"
                            background="black"
                            pad="small"
                            width= "100%">
                            <OrientationInput rotation={rotation} />
                        </Box>
                    </div>


                </Box>
            </Box>

        </>




    )
}
