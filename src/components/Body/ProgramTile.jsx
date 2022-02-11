import React from 'react';
import { Card } from 'antd';
// import { ProgramEditor } from '../ProgramEditor';
import { Environment } from 'simple-vp';
import { Detail } from '../Detail';
import Tile from '../Tile';
import useStore from '../../stores/Store';


export const ProgramTile = (props) => {

    const highlightColor = useStore(store=>store.primaryColor)

    return (
        <div style={{height:'calc(100vh - 48pt)',paddingRight:'4pt',paddingTop:'8pt',paddingBottom:'8pt'}}>
            <Tile
                style={{ height: 'calc(100vh - 62pt)'}}
                borderWidth={3}
                internalPaddingWidth={0.1}
                header={
                    <h3 style={{margin:'10pt'}}>
                        Program Editor
                    </h3>
                }
            >
                {/* <ProgramEditor/> */}
                <div style={{height:400, width: 400}}>
                    <Environment store={useStore} highlightColor={highlightColor}/>
                </div>
                
            </Tile>
        </div>
        
    )
    // return (
    //     <div style={{height:'calc(100vh - 48pt)',paddingRight:10,paddingTop:10,paddingBottom:10}}>
    //         <Card 
    //             headStyle={{height:65, paddingTop:5}}
    //             style={{height:'100%',display:'flex',flex:1,flexDirection:'column',}}
    //             bodyStyle={{padding:0,display:'flex',flex:1,flexDirection:'column',}}
    //             title="Program Editor">
    //                 <ProgramEditor/>
    //         </Card>
    //         <Detail/>
    //     </div>
    // );
};