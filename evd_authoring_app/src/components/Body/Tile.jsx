import React from 'react';

import { ThemeContext } from "../../contexts";


export const Tile = (props) => {

    const { 
        width, 
        height, 
        children 
    } = props;

    const padding = 5;

    const outerWidth = width;
    const outerHeight = height;
    const innerWidth = outerWidth - 2 * padding;
    const innerHeight = outerHeight - 2 * padding;

    return (
        <ThemeContext.Consumer>
            { value => (
                <div
                    style={{
                        padding: `${padding}px`,
                        backgroundColor: value.theme.semanticColors.bodyBackground,
                        boxShadow: '3px 3px 3px #000',
                        height: `${innerHeight}px`,
                        width: `${innerWidth}px`,
                    }}
                >
                    {children}
                </div>
            )}
        </ThemeContext.Consumer>
    );
};