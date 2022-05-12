import React, { Suspense } from 'react';
// import { Card } from 'antd';
import { Box, Spinner, Button } from 'grommet';
// import { ProgramEditor } from '../ProgramEditor';
import { Environment } from 'simple-vp';
import Tile from '../Tile';
import useStore from '../../stores/Store';
import { FiSettings, FiMaximize, FiMinimize } from "react-icons/fi";
import { TipContent } from '../TipContent';
import useMeasure from 'react-use-measure';
import { useSpring, animated } from '@react-spring/web';
import { config } from 'react-spring';

export const ProgramTile = ({ visible }) => {

    const highlightColor = useStore(state => state.primaryColor);
    const setViewMode = useStore(state => state.setViewMode);
    const viewMode = useStore(state => state.viewMode);
    const setActiveModal = useStore(state => state.setActiveModal);
    const isProcessing = useStore(state => state.processes.planProcess !== null && state.processes.planProcess !== undefined);
    const [ref, bounds] = useMeasure();

    // console.log(visible)
    if (!visible) {
        return null
    }
    return (
        <Box ref={ref} animation='fadeIn' style={{ flex: 55 }} direction='column' width='100%' height='100%' pad={{ right: '4pt', top: '4pt', bottom: '4pt', left: viewMode === 'default' ? '0pt' : '4pt' }}>
            <Tile
                style={{ height: bounds.height - 10 }}
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
                            <Button
                                tip={{
                                    content: <TipContent message={viewMode === 'default' ? 'Expand' : 'Shrink'} inverted />,
                                    plain: true,
                                    dropProps: {
                                        align: { top: 'bottom' }
                                    }
                                }}
                                icon={viewMode === 'default' ? <FiMaximize /> : <FiMinimize />}
                                onClick={() => setViewMode(viewMode === 'default' ? 'program' : 'default')}
                            />
                            <Button plain margin={{ right: 'medium' }} secondary icon={<FiSettings />} label='Settings' onClick={() => setActiveModal('settings')} />
                        </Box>
                    </Box>

                }
            >   
                <Environment store={useStore} highlightColor={highlightColor} height={bounds.height - 72} snapToGrid={false}/>
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