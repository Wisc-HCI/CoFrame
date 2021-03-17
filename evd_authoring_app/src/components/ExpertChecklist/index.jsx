import React from 'react';

import { 
    ThemeContext, 
    FrameContext 
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
                                    borderColor: themeValue.frameStyles.colors[frameValue.frame],
                                    height: '100%',
                                    width: '100%'
                                }}
                            >
                                Content
                            </div>
                        </div>
                    )}
                </FrameContext.Consumer>
            )}
        </ThemeContext.Consumer>
    );
}
