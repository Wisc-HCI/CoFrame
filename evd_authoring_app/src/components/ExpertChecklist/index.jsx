import React from 'react';

import { 
    ThemeContext, 
    FrameContext,
    ControlsContext
} from '../../contexts';

// Fill with the four possible checklist


export const ExpertChecklist = (props) => {

    const { 
        width, 
        height 
    } = props;

    const padding = 5;
    const innerWidth = width - 2 * padding;
    const innerHeight = height - 2 * padding;

    return (
        <ThemeContext.Consumer>
            { themeValue => (
                <FrameContext.Consumer>
                    { frameValue => (
                        <ControlsContext.Consumer>
                            { controlsValue => (
                                <div 
                                    style={{
                                        width: `${innerWidth}px`,
                                        height: `${innerHeight}px`
                                    }}
                                >
                                    <div 
                                        style={{
                                            borderStyle: 'solid',
                                            borderWidth: '2px',
                                            borderColor: controlsValue.inSetup ? undefined : themeValue.frameStyles.colors[frameValue.frame],
                                            height: '100%',
                                            width: '100%'
                                        }}
                                    >
                                        Content
                                        {controlsValue.inSetup ? 'true' : 'false'}
                                    </div>
                                </div>
                            )}
                        </ControlsContext.Consumer> 
                    )}
                </FrameContext.Consumer>
            )}
        </ThemeContext.Consumer>
    );
}
