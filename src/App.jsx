import React from "react";
import { FiSettings } from "react-icons/fi";
import { ReviewTile } from "./components/Body/ReviewTile";
import { SimulatorTile } from "./components/Body/SimulatorTile";
import { ProgramTile } from "./components/Body/ProgramTile";
import { Grommet, Header, Heading, Box, Button, Collapsible, Spinner } from 'grommet';

// import { Modals } from "./components/Modals";
import { Detail } from './components/Detail';
import { SettingsModal } from "./components/Settings";

import { CoFrameIcon } from "./components/Icon";

import useStore from "./stores/Store";
import shallow from 'zustand/shallow';

export default function App() {

    const setActiveModal = useStore(state => state.setActiveModal);
    const [frame, primaryColor] = useStore(state => [state.frame, state.primaryColor], shallow);
    const simMode = useStore(state => state.simMode);
    // const simStyle = useSpring({ width: simMode === 'default' ? '45%' : '100%', config: config.stiff });
    // const editStyle = useSpring({ width: simMode === 'default' ? '55%' : '0%', config: config.stiff });
    const programName = useStore(state => Object.values(state.programData).filter(v => v.type === 'programType')[0].name);
    const performPoseProcess = useStore(state=>state.performPoseProcess);
    const performPlanProcess = useStore(state=>state.performPlanProcess);
    const isProcessing = useStore(state=>state.processes.planProcess!==null&&state.processes.planProcess!==undefined);


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
        }
    }

    const menuItems = [
        {
            modalKey: 'settings',
            name: 'Settings',
            icon: <FiSettings />
        }
    ];

    return (
        <Grommet full theme={theme}>
            {/* Main container */}
            <Box direction="column" height='100vh' width='100vw'>
                <Header direction='row' pad='none' background='rgb(31,31,31)' justify='between' align='center'>
                    <Box direction='row' align='center' gap='medium' pad={{ left: 'small' }}>
                        <CoFrameIcon/>
                        <Heading level={4}><b>CoFrame<i> - {programName}</i></b></Heading>
                    </Box>
                    <Box flex></Box>
                    <Box direction='row' align='center' gap='small'>
                        {isProcessing && (
                            <Spinner/>
                        )}
                        {menuItems.map(entry => (
                            <Button plain margin={{right:'medium'}} key={entry.modalKey} secondary icon={entry.icon} label={entry.name} onClick={() => setActiveModal(entry.modalKey)}/>
                        ))}
                        {/* <Button plain label='Loc Test' margin={{right:'medium'}} key='locplanstupid' onClick={()=>performPoseProcess('location-c540bea6-a0a8-40c2-8fcc-cb6ae772697c')}/>
                        <Button plain label='Plan Test' margin={{right:'medium'}} key='planstupid' onClick={performPlanProcess}/> */}
                    </Box>
                </Header>
                <Box flex direction='row'>
                    <Box width='350pt'>
                        <ReviewTile/>
                    </Box>

                    <Box fill direction='row'>
                        {/* <animated.div style={{ ...simStyle, float: 'left' }}>
                        
                    </animated.div> */}
                    <SimulatorTile />
                    <Collapsible direction="horizontal" open={simMode==='default'}>
                        <ProgramTile />
                    </Collapsible>
                    {/* <animated.div style={{ ...editStyle, float: 'right' }}>
                        
                    </animated.div> */}
                    </Box>
                </Box>

            </Box>
            <SettingsModal/>
            <Detail/>
            {/* <Modals /> */}
        </Grommet >

    )
}