import React from 'react';

import { Separator } from '@fluentui/react/lib/Separator';


export const TileHeader = (props) => {

    const { 
        children, 
        title
    } = props;

    return (
        <div className="tile-title"
            style={{
                fontSize: '25px',
                textAlign: 'center',
                fontStyle: 'italic',
                position: 'relative'
            }}
        >
            {title}

            <Separator />

            {children}
        </div>
    );
};