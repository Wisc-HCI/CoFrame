import React from 'react';
import Tile from '../Tile';
import { Box } from 'grommet';
import { Scene } from 'robot-scene';
import useStore from '../../stores/Store';
import { Controls } from '../Controls';
import { InfoTile } from './InfoTile';
import useMeasure from 'react-use-measure';
import { useSpring, animated } from '@react-spring/web';
import { config } from 'react-spring';


export const SimulatorTile = ({ visible }) => {

    const primaryColor = useStore(state => state.primaryColor);
    const clearFocus = useStore(state => state.clearFocus);
    const tfVisible = useStore(state => state.tfVisible);
    const paused = useStore(state => state.focus === []);
    const [ref, bounds] = useMeasure();
    const containerStyle = useSpring({ flex: visible ? 11 : 0, config: config.stiff });
    
    if (!visible) {
        return null
    }
    return (
        <Box ref={ref} animation='fadeIn' flex style={{ flex: 45 }} direction='column' width='100%' height='100vh' pad='4pt' gap='xsmall'>
            <Tile
                style={{ height: bounds.height * 0.55, paddingBottom: '4pt' }}
                backgroundColor={primaryColor}
                borderWidth={3}
                internalPaddingWidth={2}
                header={
                    <Box direction='row' justify='between'>
                        <h3 style={{ margin: '10pt' }}>
                            Simulator
                        </h3>
                        <Controls />
                    </Box>
                }
            >
                <div style={{ height: bounds.height * 0.55 - 55, width: '100%', backgroundColor: 'black', padding: 0 }}>
                    <Scene
                        displayTfs={tfVisible}
                        displayGrid={true}
                        isPolar={false}
                        backgroundColor='#1e1e1e'
                        planeColor='#141414'
                        highlightColor={primaryColor}
                        plane={-0.75}
                        fov={50}
                        store={useStore}
                        onPointerMissed={clearFocus}
                        paused={paused}
                    />
                </div>

            </Tile>
            <InfoTile maxHeight={bounds.height * 0.45 - 32} />
        </Box>



    )

    return (<Box direction='column' flex pad='6pt' >
        <Tile
            style={{ height: '408pt', marginBottom: 10 }}
            backgroundColor={primaryColor}
            borderWidth={3}
            internalPaddingWidth={2}
            header={
                <Box direction='row' justify='between'>
                    <h3 style={{ margin: '10pt' }}>
                        Simulator
                    </h3>
                    <Controls />
                </Box>
            }
        >
            <div style={{ height: '364pt', width: '100%', backgroundColor: 'black', padding: 0 }}>
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
                    onPointerMissed={clearFocus}
                    paused={paused}
                />
            </div>
        </Tile>
        <InfoTile />
    </Box>)

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
