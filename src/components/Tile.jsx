import React from 'react';
import { useSpring, animated } from "@react-spring/web";
import { config } from "react-spring";


export default function Tile({style, header, children, backgroundColor, borderWidth, internalPaddingWidth}) {

    const innerContainerStyle = useSpring({
        backgroundColor:backgroundColor?backgroundColor:'rgba(0,0,0,0.6)',
        padding: internalPaddingWidth !== null ? internalPaddingWidth : 10,
        config: config.molasses
    })

    return (
        <div style={{backgroundColor:'rgba(100,100,100,0.3)', padding: borderWidth, borderRadius: 3,...style}}>
            {header && (
                <div style={{paddingBottom:borderWidth,width:'100%'}} justify='space-between'>
                    {header}
                </div>
            )}
            <animated.div style={{...innerContainerStyle, borderRadius: 3, color:'white'}}>
                {children}
            </animated.div>
        </div>
    )
}