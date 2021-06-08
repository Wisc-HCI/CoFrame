import React, { useEffect } from "react";
import { Layout, Row, Col, Button } from 'antd';
import { FolderOpenOutlined, SaveOutlined, 
         UploadOutlined, DownloadOutlined, 
         SettingOutlined } from '@ant-design/icons';
import './themes/safety.less';
import './themes/quality.less';
import './themes/performance.less';
import './themes/business.less';
import { ChecklistTile } from "./components/Body/ChecklistTile";
import { SimulatorTile } from "./components/Body/SimulatorTile";
import { ProgramTile } from "./components/Body/ProgramTile";

import { Modals } from "./components/Modals";

import useApplicationStore from "./stores/ApplicationStore";
import useGuiStore from "./stores/GuiStore";

import Logo from './content/logo.svg';


export function App(props) {

    const filename = useApplicationStore(state=>state.filename);
    const setActiveModal = useGuiStore(state=>state.setActiveModal);
    const frame = useGuiStore(state=>state.frame);
    const simMode = useGuiStore(state=>state.simMode);

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
            modalKey: 'open',
            name: 'Open',
            icon: <FolderOpenOutlined />
        },
        {
            modalKey: 'save',
            name: 'Save',
            icon: <SaveOutlined />
        },
        {
            modalKey: 'upload',
            name: 'Upload',
            icon: <UploadOutlined />
        },
        {
            modalKey: 'download',
            name: 'Download',
            icon: <DownloadOutlined />
        },
        {
            modalKey: 'settings',
            name: 'Settings',
            icon: <SettingOutlined />
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
                        <h2 style={{paddingLeft:20}}><b>Expert View Dashboard<i> - {filename}</i></b></h2>
                        <span style={{float:'right'}} >
                        {menuItems.map(entry => (
                            <Button type='text' icon={entry.icon} onClick={()=>{console.log(entry.modalKey);setActiveModal(entry.modalKey)}}>
                                {entry.name}
                            </Button>
                        ))}
                        </span>
                        
                    </Row>
                </Layout.Header>
                <Layout>
                    <Layout.Sider width='25vw'>
                        <ChecklistTile/>
                    </Layout.Sider>
                    <Layout width='75vw'>
                        <Layout.Content>
                            <Row style={{height:'100%'}}>
                                <Col style={{width:simMode==='default'? '45%' : '100%'}}>
                                    <SimulatorTile/>
                                </Col>
                                {simMode==='default' && (
                                    <Col style={{width:'55%'}}>
                                        <ProgramTile/>
                                    </Col>
                                )}
                            </Row>
                        </Layout.Content>
                    </Layout>
                </Layout>
            </Layout>

            <Modals/>
        </>
    )
}