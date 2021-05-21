import React from 'react';

import { Separator } from '@fluentui/react/lib/Separator';

import { Tile } from './Tile';
import { TileHeader } from './TileHeader';
import { Simulator } from '../Simulator';
import { Controls } from '../Controls';


export const SimulatorTile = (props) => {

    const {
        mainPadding, 
        layoutSimulator,
        checklistExists
    } = props;

    return (
        <div
            style={{
                paddingLeft: checklistExists ? `${mainPadding / 2}px` : '0',
                paddingRight: `${mainPadding / 2}px`,
                paddingTop: `${mainPadding}px`,
                paddingBottom: `${mainPadding}px`
            }}
        >

            <Tile
                width={layoutSimulator.width - mainPadding}
                height={layoutSimulator.height - mainPadding}
            >
                <TileHeader 
                    title="Simulation"
                    width={layoutSimulator.header.width - 2 * mainPadding}
                    height={layoutSimulator.header.height}
                />

                <div 
                    style={{
                        width: layoutSimulator.body.width - 2 * mainPadding,
                        height: layoutSimulator.body.height,
                    }}
                >
                    <Simulator 
                        width={layoutSimulator.body.unity.width - 3 * mainPadding}
                        height={layoutSimulator.body.unity.height}
                    />

                    <Separator />

                    <Controls 
                        width={layoutSimulator.body.controls.width - 3 * mainPadding}
                        height={layoutSimulator.body.controls.height}
                    />

                </div>
            </Tile>     
        </div>
    );
};