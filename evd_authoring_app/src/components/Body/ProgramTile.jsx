import React from 'react';

import { Card } from 'antd';
import { ProgramEditor } from '../ProgramEditor';
import { Detail } from '../Detail';




export const ProgramTile = (props) => {

    // const {editorPane, setEditorPane} = useStore(state=>({
    //     editorPane:state.editorPane,
    //     setEditorPane:state.setEditorPane
    // }))

    return (
        <div style={{height:'calc(100vh - 48pt)',paddingRight:10,paddingTop:10,paddingBottom:10}}>
            <Card 
                headStyle={{height:65, paddingTop:5}}
                style={{height:'100%',display:'flex',flex:1,flexDirection:'column',}}
                bodyStyle={{padding:0,display:'flex',flex:1,flexDirection:'column',}}
                title="Program Editor">
                    <ProgramEditor/>
            </Card>
            <Detail/>
        </div>
    );
};