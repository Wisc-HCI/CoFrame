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

import { Modals } from "./components/Modals";

import useApplicationStore from "./stores/ApplicationStore";

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
    const { 
        theme, 
        frameStyles, 
        toggleTheme, 
        themeName
    } = props;

    const {mounted, setMounted} = useState(false);
    const {modalState, setModalState} = useState({ 'settings': true });

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
                    <h2 style={{paddingLeft:20,color:theme.semanticColors.bodyText}}><b><i>Expert View Dashboard - {filename}</i></b></h2>
                    <p>TODO: ADD BACK MODAL BUTTONS</p>
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
