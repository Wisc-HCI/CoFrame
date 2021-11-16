import React, { useEffect } from "react";
import { useSpring, animated } from '@react-spring/web';
import { config } from 'react-spring';
import { Layout, Row, Button, Badge, Space } from 'antd';
import Icon, { SyncOutlined, SettingOutlined } from '@ant-design/icons';
import { ReactComponent as EvdIcon } from './components/CustomIcons/EVD.svg';
import 'antd/dist/antd.dark.css';
import './themes/safety.less';
import './themes/quality.less';
import './themes/performance.less';
import './themes/business.less';
import { ReviewTile } from "./components/Body/ReviewTile";
import { SimulatorTile } from "./components/Body/SimulatorTile";
import { ProgramTile } from "./components/Body/ProgramTile";

import { Modals } from "./components/Modals";

import useStore from "./stores/Store";
import shallow from 'zustand/shallow';

export default function App() {

    const setActiveModal = useStore(state => state.setActiveModal);
    const [frame, primaryColor] = useStore(state => [state.frame, state.primaryColor], shallow);
    const simMode = useStore(state => state.simMode);
    const simStyle = useSpring({ width: simMode === 'default' ? '45%' : '100%', config: config.stiff });
    const editStyle = useSpring({ width: simMode === 'default' ? '55%' : '0%', config: config.stiff });
    const connection = useStore(state => state.connection);
    const programName = useStore(state => Object.values(state.data).filter(v => v.type === 'program')[0].name);

    useEffect(() => {
        if (frame === 'safety') {
            document.body.classList.remove('business');
            document.body.classList.remove('performance');
            document.body.classList.remove('quality');
            document.body.classList.add('safety');
        } else if (frame === 'quality') {
            document.body.classList.remove('business');
            document.body.classList.remove('performance');
            document.body.classList.remove('safety');
            document.body.classList.add('quality');
        } else if (frame === 'performance') {
            document.body.classList.remove('business');
            document.body.classList.remove('quality');
            document.body.classList.remove('safety');
            document.body.classList.add('performance');
        } else if (frame === 'business') {
            document.body.classList.remove('performance');
            document.body.classList.remove('quality');
            document.body.classList.remove('safety');
            document.body.classList.add('business');
        }
    }, [frame]
    )

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
        <Layout style={{ height: '100vh', width: '100vw' }}>
            <Row style={{ height: '50pt', backgroundColor: 'rgb(31,31,31)', paddingLeft:30, paddingRight:30 }} align='middle' justify='space-between'>
                <Space style={{ float: 'left' }} >
                    <Icon style={{ color: primaryColor, fontSize: 30 }} component={EvdIcon} />
                    <h2 style={{ paddingLeft: 20, paddingTop: 10 }}><b>CoFrame<i> - {programName}</i></b></h2>
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
            <Row style={{ height: 'calc(100vh - 50pt)', backgroundColor: 'black' }}>
                {/* Sidebar Content */}
                <div style={{height:'100%',width:'300pt',backgroundColor: 'rgb(31,31,31)'}}>
                    <ReviewTile />
                </div>
                {/* Main App Content */}
                <div style={{ height: '100%', width: 'calc(100vw - 300pt)' }}>
                    <animated.div style={{ ...simStyle, float: 'left' }}>
                        <SimulatorTile />
                    </animated.div>
                    <animated.div style={{ ...editStyle, float: 'right' }}>
                        <ProgramTile />
                    </animated.div>
                </div>
            </Row>
            <Modals />
        </Layout>
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