import React from 'react';

import styles from '../../frameStyles';

// Fill with the four possible checklist


export const ExpertChecklist = (props) => {

    const { width, height, frame } = props;

    const padding = 5;
    const innerWidth = width - 2 * padding;
    const innerHeight = height - 2 * padding;

    return (
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
                    borderColor: styles.colors[frame],
                    height: '100%',
                    width: '100%'
                }}
            >
                Content
            </div>
        </div>
    );
}
