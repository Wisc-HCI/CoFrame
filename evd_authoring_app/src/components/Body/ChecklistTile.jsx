import React from 'react';

import { Tile } from './Tile';
import { TileHeader } from './TileHeader';
import { FrameButtons } from '../FrameButtons';
import { ExpertChecklist } from '../ExpertChecklist';

import { Pivot, PivotItem } from '@fluentui/react';

import { ControlsContext } from '../../contexts';

import frameColors from '../../frameStyles';

import useGuiStore from '../../stores/GuiStore';


export const ChecklistTile = (props) => {

    const {
        mainPadding, 
        layoutChecklist
    } = props;

    const {frameId, setFrame} = useGuiStore(state=>({frameId:state.frame,setFrame:state.setFrame}));
    console.log(frameId);

    const maskWidth = layoutChecklist.body.width - mainPadding / 2;
    const maskHeight = layoutChecklist.body.height + layoutChecklist.header.height;

    const frames = [
        {
            key:'safety',
            title:'Safety',
            content: <div>Safety Frame Content</div>
        },
        {
            key:'quality',
            title:'Quality',
            content: <div>Program Quality Frame Content</div>
        },
        {
            key:'performance',
            title:'Performance',
            content: <div>Robot Performance Frame Content</div>
        },
        {
            key:'business',
            title:'Business',
            content: <div>Business Objectives Frame Content</div>
        }
    ]

    return (
        <div 
            style={{
                paddingRight: `${mainPadding / 2}px`,
                paddingTop: `${mainPadding}px`,
                position: 'relative'
            }}
        >
            
            <Tile
                width={layoutChecklist.width - mainPadding / 2}
                height={layoutChecklist.height - mainPadding}
            >
                <TileHeader
                    title="Checklist"
                    width={layoutChecklist.header.width - 1.5 * mainPadding}
                    height={layoutChecklist.header.height}
                >
                
                    <Pivot linkFormat="tabs" selectedKey={frameId} onLinkClick={(e)=>setFrame(e.props.itemKey)}>
                        {frames.map((frame,i)=>(
                            <PivotItem
                                itemKey={frame.key}
                                key={frame.key}
                                headerText={frame.title}
                                headerButtonProps={{
                                    'data-order': i,
                                    'data-title': frame.title,
                                    'styles':frame.key === frameId ? {
                                        root:{
                                            backgroundColor: frameColors[frame.key],
                                            borderColor: frameColors[frame.key]
                                        },
                                        rootHovered:{
                                            backgroundColor: frameColors[frame.key],
                                            borderColor: frameColors[frame.key]
                                        },
                                        rootPressed:{
                                            backgroundColor: frameColors[frame.key],
                                            borderColor: frameColors[frame.key]
                                        }
                                    } : {
                                        root:{
                                            color: frameColors[frame.key]
                                        },
                                        textContainer:{
                                            color: frameColors[frame.key]
                                        }
                                    }
                                }}
                            >
                                <div style={{height:'100%',borderWidth:5,color:frameColors[frame.key]}}>
                                    {frame.content}
                                </div>
                            </PivotItem>
                        ))}
                    </Pivot>
                </TileHeader>

            </Tile>

            {/*<ControlsContext.Consumer>
                { controlsValue => (
                    <div
                        style={{
                            position: 'absolute',
                            zIndex: 2,
                            background: '#000',
                            bottom: 0,
                            left: 0,
                            width: `${maskWidth}px`,
                            height: `${maskHeight}px`,
                            display: controlsValue.inSetup ? undefined : 'none',
                            opacity: 0.9
                        }}
                    ></div>
                )}
                    </ControlsContext.Consumer> */}
        </div>
    );
};