import React, { useEffect } from "react";
import { Layout, Row, Col, Button, Badge } from 'antd';
import { SyncOutlined, 
         SettingOutlined } from '@ant-design/icons';
import './themes/safety.less';
import './themes/quality.less';
import './themes/performance.less';
import './themes/business.less';
import { ReviewTile } from "./components/Body/ReviewTile";
import { SimulatorTile } from "./components/Body/SimulatorTile";
import { ProgramTile } from "./components/Body/ProgramTile";

import { Modals } from "./components/Modals";

import useGuiStore from "./stores/GuiStore";
import useRosStore from "./stores/RosStore";

import Logo from './content/logo.svg';
import useEvdStore from "./stores/EvdStore";


export function App(props) {

    const setActiveModal = useGuiStore(state=>state.setActiveModal);
    const frame = useGuiStore(state=>state.frame);
    const simMode = useGuiStore(state=>state.simMode);
    const connection = useRosStore(state=>state.connection);
    const programName = useEvdStore(state=>state.name);

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
        <>
            <Layout style={{height:'100vh',width:'100vw'}}>
                <Layout.Header className="header">
                    <img
                        src={Logo}
                        alt="EvD Logo"
                        style={{
                            float: 'left',
                            margin: '12pt',
                            height: '30pt'
                        }}
                    />

                    <Row align={'middle'} justify='space-between'>
                        <h2 style={{paddingLeft:20}}><b>Expert View Dashboard<i> - {programName}</i></b></h2>
                        <span style={{float:'right'}} >
                        {menuItems.map(entry => (
                            <Button type='text' key={entry.modalKey} icon={entry.icon} onClick={()=>setActiveModal(entry.modalKey)}>
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
                        <ReviewTile/>
                    </Layout.Sider>
                    <Layout width='75vw'>
                        <Layout.Content>
                            <Row style={{height:'100%'}} wrap={false}>
                                <Col style={{width:simMode==='default'? '45%' : '100%', transition: 'width 0.2s linear'}}>
                                    <SimulatorTile/>
                                </Col>
                                <Col hidden={simMode!=='default'} style={{width:'55%'}}>
                                    <ProgramTile/>
                                </Col>
                            </Row>
                        </Layout.Content>
                    </Layout>
                </Layout>
            </Layout>

            <Modals/>
        </>
    )
}