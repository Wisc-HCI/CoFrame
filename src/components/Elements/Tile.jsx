import React from 'react';
// import { useSpring, animated } from "@react-spring/web";
// import { config } from "react-spring";
// import { motion } from "framer-motion";

export default function Tile({style, header, children, backgroundColor, borderWidth, internalPaddingWidth, borderRadius=3,innerStyle={}}) {

    // const innerContainerStyle = useSpring({
    //     backgroundColor:backgroundColor?backgroundColor:'rgba(0,0,0,0.6)',
    //     padding: internalPaddingWidth !== null ? internalPaddingWidth : 10,
    //     config: config.molasses
    // })

    return (
        <div style={{backgroundColor:'rgb(25,25,25)', padding: borderWidth, borderRadius,...style}}>
            {header && (
                <div style={{paddingBottom:borderWidth,width:'100%'}} justify='space-between'>
                    {header}
                </div>
            )}
            <div style={{backgroundColor: backgroundColor ? backgroundColor : 'rgba(0,0,0,0.6)', padding: internalPaddingWidth, borderRadius: 3, color:'white',...innerStyle}}>
                {children}
            </div>
        </div>
    )
}