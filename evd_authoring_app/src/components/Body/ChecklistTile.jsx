import React from 'react';

import { Tile } from './Tile';
import { TileHeader } from './TileHeader';
import { FrameButtons } from '../FrameButtons';
import { ExpertChecklist } from '../ExpertChecklist';

import { ControlsContext } from '../../contexts';


export const ChecklistTile = (props) => {

    const {
        mainPadding, 
        layoutChecklist
    } = props;

    const maskWidth = layoutChecklist.body.width - mainPadding / 2;
    const maskHeight = layoutChecklist.body.height + layoutChecklist.header.height;

    return (
        <div 
            style={{
                paddingRight: `${mainPadding / 2}px`,
                paddingTop: `${mainPadding}px`,
                paddingBottom: `${mainPadding}px`,
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
                    <FrameButtons />
                </TileHeader>
                
                <ExpertChecklist 
                    width={layoutChecklist.body.width - mainPadding }
                    height={layoutChecklist.body.height - mainPadding}
                />

            </Tile>

            <ControlsContext.Consumer>
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
            </ControlsContext.Consumer>  
        </div>
    );
};