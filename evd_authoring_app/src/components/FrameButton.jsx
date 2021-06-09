import React from 'react';
import styled from 'styled-components';
import frameStyles from '../frameStyles';

export function FrameButton(props) {

    const StyledButton = styled.button`
        flex: 1;
        display: flex;
        padding-top: 2pt;
        padding-bottom: 2pt;
        border: 1px solid ${frameStyles.colors[props.frame]};
        border-radius: 2pt;
        background-color: ${props.active ? frameStyles.colors[props.frame] : 'rgba(0,0,0,0)'};
        &:hover { // apply this style when button is hovered
            background-color: ${frameStyles.colors[props.frame]}${props.active ? 'ff' : '33'};
            transition: 0.4s ease-out;
        }
    `;

    return <StyledButton onClick={props.onClick}><span style={{fontSize:'0.9em',color:props.active ? 'black' : frameStyles.colors[props.frame]}}>{props.text}</span></StyledButton>
}