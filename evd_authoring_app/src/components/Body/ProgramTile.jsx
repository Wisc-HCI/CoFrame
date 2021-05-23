import React, { useContext, useState } from 'react';

import { DefaultButton } from '@fluentui/react/lib/Button';
import { Card } from 'antd';
import { Tile } from './Tile';
import { TileHeader } from './TileHeader';
import { ProgramEditor } from '../ProgramEditor';
import { SetupEditor } from '../SetupEditor';

import useGuiStore from '../../stores/GuiStore';




export const ProgramTile = (props) => {

    const {editorPane, setEditorPane} = useGuiStore(state=>({
        editorPane:state.editorPane,
        setEditorPane:state.setEditorPane
    }))

    return (
        <div style={{height:'100%',paddingRight:10,paddingTop:10,paddingBottom:10}}>
            <Card 
                extra={
                    <DefaultButton 
                        text={editorPane === 'setup' ? "Switch to Editor" : "Switch to Setup"}
                        onClick={() => { 
                            if (editorPane === 'setup') {
                                setEditorPane('editor')
                            } else {
                                setEditorPane('setup')
                            }
                        }}
                    />
                }
                style={{height:'100%'}}
                bodyStyle={{padding:0,display:'flex',flexDirection:'column',height:'100%'}}
                title={editorPane === 'setup' ? "Program Setup" : "Program Editor"}>
                    {editorPane === 'setup' ? (
                        <SetupEditor/>
                    ) : (
                        <ProgramEditor/>
                    )}
            </Card>
        </div>
    );
};