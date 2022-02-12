import React from 'react';
// import { Card } from 'antd';
import { Box } from 'grommet';
// import { ProgramEditor } from '../ProgramEditor';
import { Environment } from 'simple-vp';
import { Detail } from '../Detail';
import Tile from '../Tile';
import useStore from '../../stores/Store';


export const ProgramTile = (props) => {

    const highlightColor = useStore(store=>store.primaryColor)

    return (
        <Box direction='column' flex pad={{right:'6pt',top:'6pt',bottom:'6pt'}} width='45vw'>
            <Tile
                style={{ height: 'calc(100vh - 70pt)'}}
                borderWidth={3}
                internalPaddingWidth={0}
                header={
                    <h3 style={{margin:'10pt'}}>
                        Program Editor
                    </h3>
                }
            >
                {/* <ProgramEditor/> */}
                <div style={{display:'contents',flex:1,height:'100%', fontSize:10}}>
                    <Environment store={useStore} highlightColor={highlightColor} height='calc(100vh - 110pt)'/>
                </div>
                
            </Tile>
        </Box>
        
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