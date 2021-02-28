import React from 'react';

import { Separator } from 'office-ui-fabric-react';

import { Tile } from './Tile';
import { Simulator } from '../Simulator';
import { Controls } from '../Controls';


export const SimulatorTile = (props) => {

    const {frame, mainPadding, theme, layoutSimulator} = props;

    return (
        <div
            style={{
                paddingLeft: `${mainPadding / 2}px`,
                paddingRight: `${mainPadding / 2}px`,
                paddingTop: `${mainPadding}px`,
                paddingBottom: `${mainPadding}px`,
            }}
        >

            <Tile
                theme={theme}
                width={layoutSimulator.width - mainPadding}
                height={layoutSimulator.height - mainPadding}
            >
                <div
                    style={{
                        fontSize: '25px',
                        textAlign: 'center',
                        width: layoutSimulator.header.width - 2 * mainPadding,
                        height: layoutSimulator.header.height
                    }}
                >
                    <i>Simulation</i>

                    <Separator />
                    
                </div>
                <div 
                    style={{
                        width: layoutSimulator.body.width - 2 * mainPadding,
                        height: layoutSimulator.body.height,
                    }}
                >
                    <Simulator 
                        frame={frame} 
                        width={layoutSimulator.body.unity.width - 3 * mainPadding}
                        height={layoutSimulator.body.unity.height}
                    />

                    <Separator />

                    <Controls 
                        frame={frame}
                        width={layoutSimulator.body.controls.width - 3 * mainPadding}
                        height={layoutSimulator.body.controls.height}
                    />

                </div>
            </Tile>     
        </div>
    );
};