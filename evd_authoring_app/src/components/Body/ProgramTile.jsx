import React, { useContext, useState } from 'react';

import { DefaultButton } from 'office-ui-fabric-react';

import { Tile } from './Tile';
import { TileHeader } from './TileHeader';
import { ProgramEditor } from '../ProgramEditor';
import { SetupEditor } from '../SetupEditor';

import { ControlsContext } from '../../contexts';


export const ProgramTile = (props) => {

    const { 
        mainPadding, 
        layoutProgram 
    } = props;

    const controlsValue = useContext(ControlsContext);

    let button = null;
    let content = null;
    let title = null;
    if (controlsValue.inSetup) {

        button = (
            <DefaultButton 
                text="Switch to Editor" 
                onClick={() => { 
                    controlsValue.changeSetupState(false); 
                }}
            />
        );

        content = (
            <SetupEditor
                width={layoutProgram.body.width - mainPadding / 2}
                height={layoutProgram.body.height - mainPadding /2}
            />
        );

        title = 'Program Setup';

    } else {

        button = (
            <DefaultButton 
                text="Switch to Setup" 
                onClick={() => { 
                    controlsValue.changeSetupState(true); 
                }}
            />
        );

        content = (
            <ProgramEditor 
                width={layoutProgram.body.width - mainPadding / 2}
                height={layoutProgram.body.height - mainPadding /2}
            />
        );
        
        title = 'Program Editor';
    }

    return (
        <div
            style={{
                paddingTop: `${mainPadding}px`,
                paddingBottom: `${mainPadding}px`,
                paddingLeft: `${mainPadding / 2}px`,
            }}
        >
            <Tile 
                width={layoutProgram.width - mainPadding / 2}
                height={layoutProgram.height - mainPadding}
            >
                <TileHeader
                    title={title}
                    height={layoutProgram.header.height}
                    width={layoutProgram.header.width - mainPadding / 2}
                >
                    <div 
                        style={{
                            position: 'absolute',
                            top: '3px',
                            right: '13px',
                            zIndex: 1
                        }}
                    >
                        {button}    
                    </div>
                </TileHeader>

                {content}
                
            </Tile>  
        </div>
    );
};