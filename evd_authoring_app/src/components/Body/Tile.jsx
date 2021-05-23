import React from 'react';

import { ThemeContext } from "../../contexts";


export const Tile = (props) => {

    const { 
        children 
    } = props;

    const padding = 5;
    
    return (
        <ThemeContext.Consumer>
            { value => (
                <div
                    style={{
                        padding: `${padding}px`,
                        backgroundColor: value.theme.semanticColors.bodyBackground,
                        boxShadow: '3px 3px 3px #000'
                    }}
                >
                    {children}
                </div>
            )}
        </ThemeContext.Consumer>
    );
};