import React from 'react';

import { ThemeContext } from "../../contexts";


export const ElementTile = (props) => {

    const { children, style } = props;

    return (
        <ThemeContext.Consumer>
            { value => (
                <div
                    style={{
                        ...style,
                        padding: '5px',
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