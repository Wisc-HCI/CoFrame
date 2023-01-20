import React, {forwardRef} from 'react';
import { Environment } from 'simple-vp';
import Tile from '../Elements/Tile';
import useStore from '../../stores/Store';
import { FiSettings, FiMaximize, FiMinimize } from "react-icons/fi";
import { Stack, CircularProgress, IconButton, Typography } from '@mui/material';
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
        <Stack ref={ref} direction='column' style={{width:'100%',height:'100%'}} >

            <Tile
                style={{ height: '100%'}}
                borderWidth={3}
                borderRadius={0}
                internalPaddingWidth={0}
                innerStyle={{height:'calc(100% - 55px)'}}
                header={
                    <Stack direction='row' style={{paddingRight:'4px', alignContent:'center', justifyContent:'space-between'}}>
                        <Typography style={{ margin: "10pt", color:'white'}}>Program Editor</Typography>
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
                    </Stack>

                }
            >   
                <Environment store={useStore} highlightColor={highlightColor} snapToGrid={false} animateDrawer={true}/>
            </Tile>
        </Stack>
    )
});