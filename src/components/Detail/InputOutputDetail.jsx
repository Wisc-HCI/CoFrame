import React from 'react';
import useStore from '../../stores/Store';
import {Text, Box} from 'grommet';
import OrientationInput from './OrientationInput';
import PositionInput from './PositionInput';



export const ProcessIOPositionRotationDetail = ({ inputOutputId }) => {
    const position = useStore(state => {
        return state.programData[inputOutputId].properties.position;
     });
    
    const rotation = useStore(state => {
        return state.programData[inputOutputId].properties.rotation;
     });

     console.log("position: ", position );
     console.log("rot: ", rotation);

    return (
        <>
            <Box
                round="xsmall"
                background="#303030"
                pad="small" >
                <Box direction='column'>
                    <b style={{ color: 'rgba(255, 255, 255, 0.85)', paddingBottom: '2%' }} >Properties : </b>
                    <div style={{ paddingTop: '4%' }}>
                        <Box background="grey" round="small" pad="xsmall">
                            <div style={{ display: 'flex', flexDirection: 'row', justifyContent: "center", justifyItems: "center" }}>
                                <Text style={{ paddingRight: '42%' }}>Placement :</Text>
                            </div>
                            <div style={{ display: 'flex', flexDirection: 'column', alignItems: 'flex-start', paddingLeft: '30%' }}>
                                <PositionInput position={position} />
                                <OrientationInput rotation={rotation} />
                            </div>

                            <br />
                        </Box>
                    </div>


                </Box>
            </Box>

        </>

    )
}
