import React from 'react';
// import { Card } from 'antd';
import { Box, Spinner, Button } from 'grommet';
// import { ProgramEditor } from '../ProgramEditor';
import { Environment } from 'simple-vp';
import Tile from '../Tile';
import useStore from '../../stores/Store';
import { FiSettings } from "react-icons/fi";
import useMeasure from 'react-use-measure';

export const ProgramTile = (props) => {

    const highlightColor = useStore(state => state.primaryColor);
    const setActiveModal = useStore(state => state.setActiveModal);
    const isProcessing = useStore(state => state.processes.planProcess !== null && state.processes.planProcess !== undefined);
    const [ ref, bounds ] = useMeasure();
    
    return (
        <Box ref={ref} flex direction='column' width='45vw' pad={{right:'4pt',top:'4pt',bottom:'4pt'}}>
            <Tile
                style={{ height:bounds.height-10}}
                borderWidth={3}
                internalPaddingWidth={0}
                header={
                    <Box direction='row' justify='between'>
                        <h3 style={{ margin: '10pt' }}>
                            Program Editor
                        </h3>
                        <Box direction='row' align='center' gap='small'>
                            {isProcessing && (
                                <Spinner />
                            )}
                            <Button plain margin={{ right: 'medium' }} secondary icon={<FiSettings />} label='Settings' onClick={() => setActiveModal('settings')} />
                        </Box>
                    </Box>

                }
            >
                {/* <ProgramEditor/> */}
                {/* <div style={{ display: 'contents', flex: 1, height: bounds.height-50, fontSize: 10, backgroundColor:'red' }}>
                    
                </div> */}
                <Environment store={useStore} highlightColor={highlightColor} height={bounds.height-72} />

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