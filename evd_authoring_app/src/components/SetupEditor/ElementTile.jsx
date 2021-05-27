import React from 'react';


export const ElementTile = (props) => {

    const { children, style } = props;

    return (
        <div
            style={{
                ...style,
                padding: '5px',
                boxShadow: '3px 3px 3px #000'
            }}
        >
            {children}
        </div>
    );
};