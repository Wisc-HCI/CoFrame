import React, {useState} from 'react';
import { useSpring, animated } from '@react-spring/web';
import { config } from 'react-spring';
// import { RightOutlined } from '@ant-design/icons';
// import { Row, Empty } from 'antd';
import { Collapsible, Box } from 'grommet';
import { ExpandCarrot } from './ExpandCarrot';

export default function Collapse({defaultOpen, extra, style, header, children, backgroundColor, borderWidth, internalPaddingWidth}) {

    const [open, setOpen] = useState(defaultOpen)

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
                        <ExpandCarrot expanded={open} onClick={()=>setOpen(!open)}/>
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