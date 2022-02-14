import React, { useEffect } from "react";
import { useSpring, animated } from '@react-spring/web';
import { config } from 'react-spring';
// import { Layout, Row, Button, Badge, Space } from 'antd';
import Icon, { SyncOutlined, SettingOutlined } from '@ant-design/icons';
import { ReactComponent as EvdIcon } from './components/CustomIcons/EVD.svg';
// import 'antd/dist/antd.dark.css';
// import './themes/safety.less';
// import './themes/quality.less';
// import './themes/performance.less';
// import './themes/business.less';
import { ReviewTile } from "./components/Body/ReviewTile";
import { SimulatorTile } from "./components/Body/SimulatorTile";
import { ProgramTile } from "./components/Body/ProgramTile";
import { Grommet, Header, Heading, Box, Button, Collapsible } from 'grommet';

import { Modals } from "./components/Modals";

import useStore from "./stores/Store";
import shallow from 'zustand/shallow';

export default function App() {

    const setActiveModal = useStore(state => state.setActiveModal);
    const [frame, primaryColor] = useStore(state => [state.frame, state.primaryColor], shallow);
    const simMode = useStore(state => state.simMode);
    // const simStyle = useSpring({ width: simMode === 'default' ? '45%' : '100%', config: config.stiff });
    // const editStyle = useSpring({ width: simMode === 'default' ? '55%' : '0%', config: config.stiff });
    const connection = useStore(state => state.connection);
    const programName = useStore(state => Object.values(state.programData).filter(v => v.type === 'programType')[0].name);

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
        textInput: { disabled: { opacity: 1 } }
    }

    // useEffect(() => {
    //     if (frame === 'safety') {
    //         document.body.classList.remove('business');
    //         document.body.classList.remove('performance');
    //         document.body.classList.remove('quality');
    //         document.body.classList.add('safety');
    //     } else if (frame === 'quality') {
    //         document.body.classList.remove('business');
    //         document.body.classList.remove('performance');
    //         document.body.classList.remove('safety');
    //         document.body.classList.add('quality');
    //     } else if (frame === 'performance') {
    //         document.body.classList.remove('business');
    //         document.body.classList.remove('quality');
    //         document.body.classList.remove('safety');
    //         document.body.classList.add('performance');
    //     } else if (frame === 'business') {
    //         document.body.classList.remove('performance');
    //         document.body.classList.remove('quality');
    //         document.body.classList.remove('safety');
    //         document.body.classList.add('business');
    //     }
    // }, [frame]
    // )

    const menuItems = [
        {
            modalKey: 'settings',
            name: 'Settings',
            icon: <SettingOutlined />
        },
        {
            modalKey: 'sync',
            name: 'Sync',
            icon: <SyncOutlined />
        }
    ];

    return (
        <Grommet full theme={theme}>
            {/* Main container */}
            <Box direction="column" height='100vh' width='100vw'>
                <Header direction='row' pad='none' background='rgb(31,31,31)' justify='between' align='center'>
                    <Box direction='row' align='center' gap='medium' pad={{ left: 'small' }}>
                        <Icon style={{ color: primaryColor, fontSize: 30 }} component={EvdIcon} />
                        <Heading level={4}><b>CoFrame<i> - {programName}</i></b></Heading>
                    </Box>
                    <Box flex></Box>
                    <Box direction='row' align='center' gap='small'>
                        {menuItems.map(entry => (
                            <Button key={entry.modalKey} icon={entry.icon} onClick={() => setActiveModal(entry.modalKey)}>
                                {entry.name}
                            </Button>
                        ))}
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
            
            {/* <Modals /> */}
        </Grommet >

    )

    return (
        <>
            <Layout style={{ height: '100vh', width: '100vw' }}>
                <Layout.Header className="header">
                    <Row align='middle' justify='space-between'>
                        <Space style={{ float: 'left' }} >
                            <Icon style={{ color: primaryColor, fontSize: 30 }} component={EvdIcon} />
                            <h2 style={{ paddingLeft: 20 }}><b>CoFrame<i> - {programName}</i></b></h2>
                        </Space>
                        <span style={{ float: 'right' }} >
                            {menuItems.map(entry => (
                                <Button type='text' key={entry.modalKey} icon={entry.icon} onClick={() => setActiveModal(entry.modalKey)}>
                                    {entry.name}
                                </Button>
                            ))}
                            {connection === 'connected' && (
                                <Badge status="success" />
                            )}
                            {connection === 'connecting' && (
                                <Badge status="warning" />
                            )}
                            {connection === 'disconnected' && (
                                <Badge status="error" />
                            )}
                        </span>
                    </Row>
                </Layout.Header>
                <Layout>
                    <Layout.Sider width='25vw'>
                        <ReviewTile />
                    </Layout.Sider>
                    <Layout width='75vw'>
                        <Layout.Content>
                            <div style={{ height: '100%', width: '100%' }}>
                                <div style={{ width: '45%', float: 'left' }}>
                                    <SimulatorTile />
                                </div>
                                <div style={{ width: '55%', float: 'right' }}>
                                    <ProgramTile />
                                </div>
                            </div>
                        </Layout.Content>
                    </Layout>
                </Layout>
            </Layout>

            <Modals />
        </>
    )
}