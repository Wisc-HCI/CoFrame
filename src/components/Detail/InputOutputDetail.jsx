import React from 'react';
import useStore from '../../stores/Store';
import { Text, Box } from 'grommet';
import RotationInput from './RotationInput';
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
           
        </>




    )
}
