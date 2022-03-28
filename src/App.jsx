import React from "react";
import { FiSettings } from "react-icons/fi";
import { ReviewTile } from "./components/Body/ReviewTile";
import { SimulatorTile } from "./components/Body/SimulatorTile";
import { ProgramTile } from "./components/Body/ProgramTile";
import { Grommet, Header, Heading, Box, Button, Collapsible, Spinner } from 'grommet';
import ParentSize from "@visx/responsive/lib/components/ParentSize";
import { TIMELINE_TYPES } from "./stores/Constants";
// import { Modals } from "./components/Modals";
import { Detail } from './components/Detail';
import { SettingsModal } from "./components/Settings";
import Graph from "./components/Graph";

// import { CoFrameIcon } from "./components/Icon";

import useStore from "./stores/Store";

import { useSpring, animated } from '@react-spring/web';
import { config } from 'react-spring';
// import { Timeline } from "./components/Timeline";

export default function App() {

    const primaryColor = useStore(state => state.primaryColor);
    const simMode = useStore(state => state.simMode);
    // const simStyle = useSpring({ width: simMode === 'default' ? '45%' : '100%', config: config.stiff });
    // const editStyle = useSpring({ width: simMode === 'default' ? '55%' : '0%', config: config.stiff });
    const visibleSteps = useStore(state => state.focus.some(focusItem => TIMELINE_TYPES.includes(state.programData[focusItem]?.type)));

    const bodyStyle = useSpring({ width: '100vw', height: visibleSteps ? '80vh' : '100vh', config: config.stiff });

    const theme = {
        name: 'CoFrame',
        rounding: 4,
        defaultMode: 'dark',
        global: {
            colors: {
                brand: primaryColor,
                background: '#111111',
                control: primaryColor
            },
            font: {
                family: "Helvetica"
            },
            focus: {
                border: {
                    color: primaryColor
                }
            },
            input: {
                padding: 4,
                extend: { backgroundColor: '#FFFFFF55' }
            },
            // edgeSize: {large: 50, small: 10, medium: 15}
        },
        button: {
            border: {
                radius: "4px"
            }
        },
        radioButton: {
            size: "16px",
            border: { color: '#00000088' }
        },
        checkBox: {
            size: "20px",
            border: { color: '#00000088' },
            color: primaryColor,
            hover: { border: { color: '#00000088' }, }
        },
        textInput: {
            disabled: { opacity: 1 }
        },
        notification: {
            toast: {
                container: {
                    elevation: 'none'
                }
            },
            container: {
                border: { color: 'lightgrey' },
                background: {
                    color: 'background-front',
                }
            }
        },
        tab: {
            active: {
                background: primaryColor,
                color: 'dark-1'
            },
            background: 'dark-3',
            border: undefined,
            color: 'white',
            hover: {
                 background: '#444444',
                 color: 'white'
            },
            margin: undefined,
            pad: {
                bottom: undefined,
                horizontal: 'small',
            },
            extend: {
                borderRadius: 4,
                padding: 6
            }
        },
        tabs: {
            gap: 'medium',
            header: {
                extend: {padding: 10}
            },
            panel: { padding: 10 },
            extend: {padding: 10}
        }
    }

    return (
        <Grommet full theme={theme}>
            {/* Main container */}
            <Box direction="column" height='100vh' width='100vw'>
                <animated.div style={{ ...bodyStyle, width: '100vw', display: 'flex', flexDirection: 'row' }}>
                    <Box width='350pt'>
                        <ReviewTile />
                    </Box>

                    <Box fill direction='row'>
                        {/* <animated.div style={{ ...simStyle, float: 'left' }}>
                        
                    </animated.div> */}
                        <SimulatorTile />
                        <Collapsible direction="horizontal" open={simMode === 'default'}>
                            <ProgramTile />
                        </Collapsible>
                        {/* <animated.div style={{ ...editStyle, float: 'right' }}>
                        
                    </animated.div> */}
                    </Box>
                </animated.div>
                {visibleSteps && (
                    <Box direction='row' height={visibleSteps ? '20vh' : '0vh'} width='100vw' background='#444444' border={{ side: 'top', color: primaryColor, size: 'medium' }}>
                        <ParentSize>
                            {({ width, height }) => <Graph width={width} height={height - 10} />}
                        </ParentSize>
                    </Box>
                )}

            </Box>
            <SettingsModal />
            <Detail />
            {/* <Modals /> */}
        </Grommet >

    )
}