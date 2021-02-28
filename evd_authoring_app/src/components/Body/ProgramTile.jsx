import React from 'react';

import { Separator } from 'office-ui-fabric-react';

import { Tile } from './Tile';
import { ProgramEditor } from '../ProgramEditor';


export const ProgramTile = (props) => {

    const { mainPadding, theme, layoutProgram } = props;

    return (
        <div
            style={{
                paddingTop: `${mainPadding}px`,
                paddingBottom: `${mainPadding}px`,
                paddingLeft: `${mainPadding / 2}px`,
            }}
        >
            <Tile 
                theme={theme}
                width={layoutProgram.width - mainPadding / 2}
                height={layoutProgram.height - mainPadding}
            >
                <div
                    style={{
                        height: `${layoutProgram.header.height}px`,
                        width: `${layoutProgram.header.width - mainPadding / 2}px`,
                        fontSize: '25px',
                        textAlign: 'center'
                    }}
                >
                    <i>Program</i>

                    <Separator />
                    
                </div>

                <ProgramEditor 
                    width={layoutProgram.body.width - mainPadding / 2}
                    height={layoutProgram.body.height - mainPadding /2}
                />
            </Tile>  
        </div>
    );
};