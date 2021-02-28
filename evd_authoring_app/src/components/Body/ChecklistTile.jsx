import React from 'react';

import { Separator } from 'office-ui-fabric-react';

import { Tile } from './Tile';
import { FrameButtons } from '../FrameButtons';
import { ExpertChecklist } from '../ExpertChecklist';


export const ChecklistTile = (props) => {

    const {frame, mainPadding, theme, layoutChecklist, onFrameButtonCallback } = props;

    console.log(onFrameButtonCallback);

    return (
        <div style={{
                paddingRight: `${mainPadding / 2}px`,
                paddingTop: `${mainPadding}px`,
                paddingBottom: `${mainPadding}px`,
            }}
        >
            <Tile
                theme={theme}
                width={layoutChecklist.width - mainPadding / 2}
                height={layoutChecklist.height - mainPadding}
            >
                <div
                    style={{
                        width: layoutChecklist.header.width - 1.5 * mainPadding,
                        height: layoutChecklist.header.height
                    }}
                >
                    <div
                        style={{
                            fontSize: '25px',
                            textAlign: 'center',
                        }}
                    >
                        <i>Checklist</i>

                        <Separator />
                        
                    </div>

                    <FrameButtons 
                        frame={frame} 
                        callback={onFrameButtonCallback} 
                    />
                </div>
                
                <ExpertChecklist 
                    frame={frame}
                    width={layoutChecklist.body.width - mainPadding }
                    height={layoutChecklist.body.height - mainPadding}
                />

            </Tile>
        </div>
    );
};