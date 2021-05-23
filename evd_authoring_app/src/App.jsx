import React, { useState } from "react";
// import { UnityContent } from 'react-unity-webgl';
import 'antd/dist/antd.css';
import 'antd/dist/antd.dark.css';
// import { computeLayout } from "./layout";
import { Layout, Row, Col } from 'antd';

import { Header } from "./components/Header";
import { Body } from "./components/Body";
import { ChecklistTile } from "./components/Body/ChecklistTile";
import { SimulatorTile } from "./components/Body/SimulatorTile";
import { ProgramTile } from "./components/Body/ProgramTile";
import { CommandBar } from '@fluentui/react/lib/CommandBar';
import { ActionButton } from '@fluentui/react/lib/Button';

import { Modals } from "./components/Modals";

import useApplicationStore from "./stores/ApplicationStore";
import useGuiStore from "./stores/GuiStore";

import Logo from './content/logo.svg';

import { 
    GetRosServiceSingleton,
    GetApplicationServiceSingleton ,
    GetEvDScriptServiceSingleton,
    GetPendingServiceSingleton
} from './services';

import { 
    ThemeContext,
    ApplicationContext,
    RosContext,
    EvDScriptContext,
    PendingContext,
    UnityContext,
    ModalContext
} from "./contexts";


export function App(props) {

    const filename = useApplicationStore(state=>state.filename);
    const setActiveModal = useGuiStore(state=>state.setActiveModal);
    const { 
        theme, 
        frameStyles, 
        toggleTheme, 
        themeName
    } = props;

    // const {mounted, setMounted} = useState(false);
    // const {modalState, setModalState} = useState({ 'settings': true });

    const menuItems = [
        {
            key: 'open',
            name: 'Open',
            iconName: 'OpenFolderHorizontal'
        },
        {
            key: 'save',
            name: 'Save',
            iconName: 'Save'
        },
        {
            key: 'upload',
            name: 'Upload',
            iconName: 'Upload'
        },
        {
            key: 'download',
            name: 'Download',
            iconName: 'Download'
        },
        {
            key: 'settings',
            name: 'Settings',
            iconName: 'Settings'
        }
    ];

    return (
        <ThemeContext.Provider 
                value={{
                    theme: theme,
                    frameStyles: frameStyles,
                    toggleTheme: toggleTheme,
                    themeName: themeName
                }}
            >
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
                    <Row align={'middle'} justify={'space-between'}>
                        <h2 style={{paddingLeft:20,color:theme.semanticColors.bodyText}}><b>Expert View Dashboard<i> - {filename}</i></b></h2>
                        <span style={{float:'right'}} >
                        {menuItems.map(entry => (
                            <ActionButton iconProps={{iconName:entry.iconName}} allowDisabledFocus onClick={()=>setActiveModal(entry.key)}>
                                {entry.name}
                            </ActionButton>
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
                                <Col style={{width:'45%'}}>
                                    <SimulatorTile/>
                                </Col>
                                <Col style={{width:'55%'}}>
                                    <ProgramTile/>
                                </Col>
                            </Row>
                        </Layout.Content>
                    </Layout>
                </Layout>
            </Layout>

            <Modals/>         
        </ThemeContext.Provider>
    )
}
