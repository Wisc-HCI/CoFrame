import React from 'react';

import { Stack } from 'office-ui-fabric-react';

import { StyledButton } from './StyledButton';
import { FrameContext } from '../../contexts';


export const FrameButtons = (props) => {

    const frames = [
        { name: "Safety", key: "safety" },
        { name: "Program Quality", key: "quality" },
        { name: "Robot Performance", key: "performance" },
        { name: "Buisness Objectives", key: "buisness" }
    ]

    return (
        <FrameContext.Consumer>
            { frameValue => (
                <Stack horizontal>

                    {frames.map(entry => (
                        <StyledButton 
                            key={entry.key}
                            text={entry.name}
                            frame={entry.key}
                            primary={frameValue.frame === entry.key} 
                            callback={() => frameValue.changeFrame(entry.key)}
                        />
                    ))}

                </Stack>
            )}
        </FrameContext.Consumer>
    );
};