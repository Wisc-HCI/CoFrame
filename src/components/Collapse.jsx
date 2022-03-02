import React, {useState} from 'react';
import { useSpring, animated } from '@react-spring/web';
import { config } from 'react-spring';
import { RightOutlined } from '@ant-design/icons';
// import { Row, Empty } from 'antd';
import { Collapsible, Box } from 'grommet'

export default function Collapse({openable, extra, style, header, children, backgroundColor, borderWidth, internalPaddingWidth}) {

    const [open, setOpen] = useState(false)

    const carrotStyle = useSpring({
        rotate: open ? '90deg' : '0deg',
        config: config.wobbly
    });

    const contentStyle = useSpring({ 
        scaleY: open ? 1 : 0,
        opacity: open ? 1 : 0
      });

    return (
        <div style={{backgroundColor:'rgba(100,100,100,0.3)', padding: borderWidth, borderRadius: 3, ...style}}>
            {header && (
                <Box direction='row' style={{paddingBottom:borderWidth,width:'100%'}} justify='between' align='center'>
                    {header}
                    <Box direction='row' justify='end' align='center'>
                        {extra}
                        <animated.div onClick={()=>{openable && setOpen(!open)}} style={{...carrotStyle, margin:10}}><RightOutlined/></animated.div>
                    </Box>
                </Box>
            )}
            <Collapsible open={open}>
                <Box style={{
                    ...contentStyle,
                    backgroundColor:backgroundColor?backgroundColor:'rgba(0,0,0,0.6)', 
                    padding: internalPaddingWidth ? internalPaddingWidth : 10, 
                    borderRadius: 3, color:'white'
                }}>
                    {children ? children : <Box height='small' width='100%' align='center' justify='center'>No Data</Box>}
                </Box>
            </Collapsible>
            
        </div>
    )
}