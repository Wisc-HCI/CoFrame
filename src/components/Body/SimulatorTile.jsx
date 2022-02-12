import React from 'react';
import Tile from '../Tile';
import { Box } from 'grommet';
import {Scene} from 'robot-scene';
import useStore from '../../stores/Store';
import { Controls } from '../Controls';
import { InfoTile } from './InfoTile';


export const SimulatorTile = (_) => {

    const primaryColor = useStore(state => state.primaryColor);
    const clearFocusItem = useStore(state => state.clearFocusItem);
    const paused = useStore(state => state.focusItem.uuid === null);

    return (
        <Box direction='column' flex pad='6pt' >
            <Tile
                style={{ height: '408pt', marginBottom: 10 }}
                backgroundColor={primaryColor}
                borderWidth={3}
                header={
                    <Box direction='row' justify='between'>
                    <h3 style={{margin:'10pt'}}>
                        Simulator
                    </h3>
                    <Controls/>
                    </Box>
                }
            >
                <div style={{ height: '368pt', width: '100%', backgroundColor:'black', padding:0 }}>
                    <Scene
                        displayTfs={false}
                        displayGrid={true}
                        isPolar={false}
                        backgroundColor='#1e1e1e'
                        planeColor='#141414'
                        highlightColor={primaryColor}
                        plane={-0.75}
                        fov={50}
                        store={useStore}
                        onPointerMissed={clearFocusItem}
                        paused={paused}
                    />
                </div>
            </Tile>
            <InfoTile />
        </Box>
    )

    // return (
    //     <div style={{ height: 'calc(100vh - 48pt)', padding: 10 }}>
    //         <Card
    //             style={{ height: 564, marginBottom: 10 }}
    //             headStyle={{ height: 65 }}
    //             bodyStyle={{ padding: 0, height: 500, margin: 0, }}
    //             extra={<Controls />}
    //             title="Simulator">
    //             <div
    //                 style={{ height: '100%', backgroundColor: primaryColor, padding: 5, width: '100%' }}
    //             >
    //                 {/* <Scene
    //                         displayTfs={false}
    //                         displayGrid={true}
    //                         isPolar={false}
    //                         backgroundColor='#1e1e1e'
    //                         planeColor='#141414'
    //                         highlightColor={primaryColor}
    //                         plane={-0.75}
    //                         fov={50}
    //                         store={useStore}
    //                         onPointerMissed={clearFocusItem}
    //                         paused={paused}
    //                     /> */}

    //             </div>
    //         </Card>
    //         <InfoTile />
    //     </div>
    // );
};
