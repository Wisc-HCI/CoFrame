import React, { useEffect } from "react";
import { useSpring, animated } from '@react-spring/web';
import { config } from 'react-spring';
import useMeasure from 'react-use-measure'
import { Layout, Row, Button, Badge, Space } from 'antd';
import Icon, { SyncOutlined, SettingOutlined } from '@ant-design/icons';
import {ReactComponent as EvdIcon} from './components/CustomIcons/EVD.svg';
import './themes/safety.less';
import './themes/quality.less';
import './themes/performance.less';
import './themes/business.less';
import { ReviewTile } from "./components/Body/ReviewTile";
import { SimulatorTile } from "./components/Body/SimulatorTile";
import { ProgramTile } from "./components/Body/ProgramTile";

import { Modals } from "./components/Modals";

import useStore from "./stores/Store";


export function App() {

    const setActiveModal = useStore(state=>state.setActiveModal);
    const [frame,primaryColor] = useStore(state=>[state.frame,state.primaryColor]);
    const simMode = useStore(state=>state.simMode);
    const [devRef, {width}] = useMeasure();
    const simStyle = useSpring({width: simMode==='default' ? width * 0.45 : width, config:config.stiff});
    const editStyle = useSpring({width: simMode==='default' ? width * 0.55 : 0, config:config.stiff});
    const connection = useStore(state=>state.connection);
    const programName = useStore(state=>state.name);

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
                    <Row align='middle' justify='space-between'>
                        <Space style={{float:'left'}} >
                            <Icon style={{color:primaryColor,fontSize:30}} component={EvdIcon}/>
                            <h2 style={{paddingLeft:20}}><b>Expert View Dashboard<i> - {programName}</i></b></h2>
                        </Space>
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
                            <div ref={devRef} style={{height:'100%',width:'100%'}}>
                                <animated.div style={{...simStyle,float:'left'}}>
                                    <SimulatorTile/>
                                </animated.div>
                                <animated.div style={{...editStyle,float:'right'}}>
                                    <ProgramTile/>
                                </animated.div>
                                {/* <Col style={{width:simMode==='default'? '45%' : '100%', transition: 'width 0.2s linear'}}>
                                    
                                </Col>
                                <Col hidden={simMode!=='default'} style={{width:'55%'}}>
                                    
                                </Col> */}
                            </div>
                        </Layout.Content>
                    </Layout>
                </Layout>
            </Layout>

            <Modals/>
        </>
    )
}