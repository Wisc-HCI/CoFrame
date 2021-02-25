import React from 'react';

export const Tile = (props) => {

    const { theme, width, height, children } = props;

    const padding = 5;

    const outerWidth = width;
    const outerHeight = height;
    const innerWidth = outerWidth - 2 * padding;
    const innerHeight = outerHeight - 2 * padding;

    return (
        <div
            style={{
                padding: `${padding}px`,
                backgroundColor: theme.semanticColors.bodyBackground,
                boxShadow: '3px 3px 3px #000',
                height: `${innerHeight}px`,
                width: `${innerWidth}px`,
            }}
        >
            {children}
        </div>
    );
};