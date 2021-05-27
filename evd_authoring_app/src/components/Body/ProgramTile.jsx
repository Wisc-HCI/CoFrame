import React, { useContext, useState } from 'react';

import { Card, Button } from 'antd';
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
                    <Button onClick={() => { 
                        if (editorPane === 'setup') {
                            setEditorPane('editor')
                        } else {
                            setEditorPane('setup')
                        }
                    }}>
                        {editorPane === 'setup' ? "Switch to Editor" : "Switch to Setup"}
                    </Button>
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