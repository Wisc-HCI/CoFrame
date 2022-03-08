import React from 'react';
import { TextInput } from "grommet";
import { toNumber } from 'lodash';

export const NumberInput = ({ value, min, max, onChange, disabled, style, visualScaling }) => {
    const usedScaling = visualScaling ? visualScaling : 1;
    return (
        <TextInput
            disabled={disabled}
            value={value*usedScaling}
            type='number'
            step={0.1}
            textAlign="center"
            style={{fontSize:14,color: toNumber(value) >= min && toNumber(value) <= max ? style?.color : 'red'}}
            onChange={event => {onChange(toNumber(event.target.value)/usedScaling)}}
        />
    )
}