import React from 'react';
import styled from 'styled-components';
import frameStyles from '../frameStyles';
import { Button, ThemeContext } from 'grommet';

// const StyledButton = styled.button`
//     flex: 1;
//     display: flex;
//     padding-top: 2pt;
//     padding-bottom: 4pt;
//     border: 1px solid ${props => frameStyles.colors[props.frame]};
//     border-radius: 2pt;
//     background-color: ${props => (props.active ? frameStyles.colors[props.frame] : 'rgba(0,0,0,0)')};
//     &:hover { // apply this style when button is hovered
//         background-color: ${props => frameStyles.colors[props.frame]}${props => (props.active ? 'ff' : '33')};
//         transition: 0.4s ease-out;
//     }
// `;

// export function FrameButton(props) {



//     return <StyledButton frame={props.frame} active={props.active} onClick={props.onClick}><span style={{ fontSize: '0.95em', color: props.active ? 'black' : frameStyles.colors[props.frame] }}>{props.text}</span></StyledButton>
// }

export const FrameButton = ({ frame, active, onClick, text }) => {
    return (
    <ThemeContext.Extend
        value={
            {   
                global: {
                    colors: {
                        brand: frameStyles.colors[frame],
                        control: frameStyles.colors[frame]
                    },
                    focus: {
                        border: {
                            color: frameStyles.colors[frame]
                        }
                    }
                }
            }
        }
    >
        <Button size='small' round='xsmall' primary={active} label={text} onClick={onClick} style={{padding: 5,borderRadius:4,color:active?'black':frameStyles.colors[frame]}}/>
    </ThemeContext.Extend>
    )
}