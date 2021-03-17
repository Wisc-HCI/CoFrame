import React from 'react';

import { Tile } from './Tile';
import { TileHeader } from './TileHeader';
import { FrameButtons } from '../FrameButtons';
import { ExpertChecklist } from '../ExpertChecklist';


export const ChecklistTile = (props) => {

    const {
        mainPadding, 
        layoutChecklist
    } = props;

    return (
        <div 
            style={{
                paddingRight: `${mainPadding / 2}px`,
                paddingTop: `${mainPadding}px`,
                paddingBottom: `${mainPadding}px`,
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
        </div>
    );
};