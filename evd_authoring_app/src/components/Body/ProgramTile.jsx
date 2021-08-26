import React from 'react';

import { Card, Button } from 'antd';
import { ProgramEditor } from '../ProgramEditor';
import { SetupEditor } from '../SetupEditor';
import { Detail } from '../Detail';

import useGuiStore from '../../stores/GuiStore';




export const ProgramTile = (props) => {

    const {editorPane, setEditorPane} = useGuiStore(state=>({
        editorPane:state.editorPane,
        setEditorPane:state.setEditorPane
    }))

    return (
        <div style={{height:'calc(100vh - 48pt)',paddingRight:10,paddingTop:10,paddingBottom:10}}>
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
                style={{height:'100%',display:'flex',flex:1,flexDirection:'column',}}
                bodyStyle={{padding:0,display:'flex',flex:1,flexDirection:'column'}}
                title={editorPane === 'setup' ? "Program Setup" : "Program Editor"}>
                    {editorPane === 'setup' ? (
                        <SetupEditor/>
                    ) : (
                        <ProgramEditor/>
                    )}
                    <Detail/>   
            </Card>
        </div>
    );
};