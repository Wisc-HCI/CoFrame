import React from 'react';
import { Row } from 'antd';

export default function Tile({style, header, children, backgroundColor, borderWidth, internalPaddingWidth}) {

    return (
        <div style={{backgroundColor:'rgba(100,100,100,0.3)', padding: borderWidth, borderRadius: 3,...style}}>
            {header && (
                <Row style={{paddingBottom:borderWidth,width:'100%'}} justify='space-between'>
                    {header}
                </Row>
            )}
            <div style={{backgroundColor:backgroundColor?backgroundColor:'rgba(0,0,0,0.6)', padding: internalPaddingWidth ? internalPaddingWidth : 10, borderRadius: 3, color:'white'}}>
                {children}
            </div>
        </div>
    )
}