import React from 'react';

import { Separator } from '@fluentui/react/lib/Separator';


export const TileHeader = (props) => {

    const { 
        children, 
        title, 
        width, 
        height 
    } = props;

    return (
        <div className="tile-title"
            style={{
                fontSize: '25px',
                textAlign: 'center',
                fontStyle: 'italic',
                width: `${width}px`,
                height: `${height}px`,
                position: 'relative'
            }}
        >
            {title}

            <Separator />

            {children}
        </div>
    );
};