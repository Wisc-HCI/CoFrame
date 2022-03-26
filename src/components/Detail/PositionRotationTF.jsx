import React from 'react';
import useStore from '../../stores/Store';
import { Box } from "grommet";
import PositionInput from './PositionInput';
import RotationInput from './RotationInput';

function PositionRotationTF(props) {
    


    return (
        <>
            <Box
                round="xsmall"
                background="#303030"
                pad="small" wrap = {true}>
                <Box direction='column' >
                    <b style={{ color: 'rgba(255, 255, 255, 0.85)', paddingBottom: '2%' }} >Placement : </b>
                    <div style={{ display: 'flex', flexDirection: 'column' }}>
                        <div style={{ paddingBottom: "4%" }}>
                            <Box>
                            <PositionInput itemID={props.itemID} position={props.position} prevID = {props.prevID} />
                            </Box>
                        </div>
                        <Box>
                        <RotationInput rotation={props.rotation} itemID={props.itemID} prevID = {props.prevID}/>
                        </Box>
                        
                    </div>
                </Box>
            </Box>
        </>
    )

}



export default (PositionRotationTF);
