import React from 'react';

import { Tile } from './Tile';
import { TileHeader } from './TileHeader';
import { FrameButtons } from '../FrameButtons';
import { ExpertChecklist } from '../ExpertChecklist';
import { Card } from 'antd';

import { Pivot, PivotItem } from '@fluentui/react';
import { Separator } from '@fluentui/react/lib/Separator';

import frameStyles from '../../frameStyles';

import useGuiStore from '../../stores/GuiStore';


export const ChecklistTile = (_) => {

    const {frameId, setFrame} = useGuiStore(state=>({frameId:state.frame,setFrame:state.setFrame}));
    
    const frames = [
        {
            key:'safety',
            tab:'Safety',
            content: <div>Safety Frame Content</div>
        },
        {
            key:'quality',
            tab:'Program Quality',
            content: <div>Program Quality Frame Content</div>
        },
        {
            key:'performance',
            tab:'Performance',
            content: <div>Robot Performance Frame Content</div>
        },
        {
            key:'business',
            tab:'Business',
            content: <div>Business Objectives Frame Content</div>
        }
    ]

    return (
        <div style={{height:'100%',paddingLeft:10,paddingRight:10,paddingBottom:10}}>
            <Card
                style={{height:'100%'}}
                bodyStyle={{padding:0,display:'flex',flexDirection:'column',height:'100%'}}
                title="Checklist"
            >
                <Pivot style={{padding:10}} linkFormat="tabs" selectedKey={frameId} onLinkClick={(e)=>setFrame(e.props.itemKey)}>
                    {frames.map((frame,i)=>(
                        <PivotItem
                            itemKey={frame.key}
                            key={frame.key}
                            headerText={frame.tab}
                            headerButtonProps={{
                                'data-order': i,
                                'data-title': frame.tab,
                                'styles':frame.key === frameId ? {
                                    root:{
                                        backgroundColor: frameStyles.colors[frame.key],
                                        borderColor: frameStyles.colors[frame.key]
                                    },
                                    rootHovered:{
                                        backgroundColor: frameStyles.colors[frame.key],
                                        borderColor: frameStyles.colors[frame.key]
                                    },
                                    rootPressed:{
                                        backgroundColor: frameStyles.colors[frame.key],
                                        borderColor: frameStyles.colors[frame.key]
                                    }
                                } : {
                                    root:{
                                        color: frameStyles.colors[frame.key]
                                    },
                                    textContainer:{
                                        color: frameStyles.colors[frame.key]
                                    }
                                }
                            }}
                        >
                            <div style={{height:'100%',borderWidth:5,border:frameStyles.colors[frame.key]}}>
                                <Separator/>
                                {frame.content}
                            </div>
                        </PivotItem>
                    ))}
                </Pivot>
            </Card>
        </div>
        


        // <Tile>
        //     <TileHeader title="Checklist">
            
        //         <Pivot linkFormat="tabs" selectedKey={frameId} onLinkClick={(e)=>setFrame(e.props.itemKey)}>
        //             {frames.map((frame,i)=>(
        //                 <PivotItem
        //                     itemKey={frame.key}
        //                     key={frame.key}
        //                     headerText={frame.title}
        //                     headerButtonProps={{
        //                         'data-order': i,
        //                         'data-title': frame.title,
        //                         'styles':frame.key === frameId ? {
        //                             root:{
        //                                 backgroundColor: frameStyles.colors[frame.key],
        //                                 borderColor: frameStyles.colors[frame.key]
        //                             },
        //                             rootHovered:{
        //                                 backgroundColor: frameStyles.colors[frame.key],
        //                                 borderColor: frameStyles.colors[frame.key]
        //                             },
        //                             rootPressed:{
        //                                 backgroundColor: frameStyles.colors[frame.key],
        //                                 borderColor: frameStyles.colors[frame.key]
        //                             }
        //                         } : {
        //                             root:{
        //                                 color: frameStyles.colors[frame.key]
        //                             },
        //                             textContainer:{
        //                                 color: frameStyles.colors[frame.key]
        //                             }
        //                         }
        //                     }}
        //                 >
        //                     <div style={{height:'100%',borderWidth:5,color:frameStyles.colors[frame.key]}}>
        //                         {frame.content}
        //                     </div>
        //                 </PivotItem>
        //             ))}
        //         </Pivot>
        //     </TileHeader>

        // </Tile>
    );
};