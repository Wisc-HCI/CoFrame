import React, {forwardRef} from 'react';
import { Box, Spinner, Button } from 'grommet';
import { Environment } from 'simple-vp';
import Tile from '../Elements/Tile';
import useStore from '../../stores/Store';
import { FiSettings, FiMaximize, FiMinimize } from "react-icons/fi";
import { TipContent } from '../Elements/TipContent';
import useMeasure from 'react-use-measure';
import { Stack, CircularProgress, IconButton } from '@mui/material';
import ParentSize from "@visx/responsive/lib/components/ParentSize";
import shallow from 'zustand/shallow';

export const ProgramTile = forwardRef((_,ref) => {

    const highlightColor = useStore(state => state.primaryColor,shallow);
    const setViewMode = useStore(state => state.setViewMode,shallow);
    const viewMode = useStore(state => state.viewMode,shallow);
    const setActiveModal = useStore(state => state.setActiveModal,shallow);
    const isProcessing = useStore(state => state.processes.planProcess !== null && state.processes.planProcess !== undefined,shallow);
    // const [ref, bounds] = useMeasure();

    // console.log(visible)
    return (
        <Box ref={ref} animation='fadeIn' direction='column' width='100%' height='100%'>
            <ParentSize>
              {({ height }) =>
            <Tile
                style={{ height: '100%' }}
                borderWidth={3}
                borderRadius={0}
                internalPaddingWidth={0}
                header={
                    <Box direction='row' justify='between' align='center' pad={{right:'small'}}>
                        <h3 style={{ margin: '10pt' }}>
                            Program Editor
                        </h3>
                        <Stack direction='row' gap={1} alignItems='center'>
                            {isProcessing && (
                                <CircularProgress size={18} variant='indeterminate' color='primaryColor' />
                            )}
                            <IconButton size='small' onClick={() => setViewMode(viewMode === 'default' ? 'program' : 'default')}>
                                {viewMode === 'default' ? <FiMaximize /> : <FiMinimize />}
                            </IconButton>
                            <IconButton size='small' onClick={() => setActiveModal('settings')}>
                                <FiSettings />
                            </IconButton>
                        </Stack>
                        {/* <Box direction='row' align='center' gap='small'>
                            
                            <Button
                                tip={{
                                    content: <TipContent message={viewMode === 'default' ? 'Expand' : 'Shrink'} inverted />,
                                    plain: true,
                                    dropProps: {
                                        align: { top: 'bottom' }
                                    }
                                }}
                                icon=
                                onClick={() => setViewMode(viewMode === 'default' ? 'program' : 'default')}
                            />
                            <Button plain margin={{ right: 'medium' }} secondary icon={<FiSettings />} label='Settings' onClick={() => setActiveModal('settings')} />
                        </Box> */}
                    </Box>

                }
            >   
            
                <Environment store={useStore} highlightColor={highlightColor} height={height-62} snapToGrid={false} animateDrawer={false}/>
             
                
            </Tile>
             }
             </ParentSize>
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
});