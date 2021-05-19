import React, { Component } from "react";
import { UnityContent } from 'react-unity-webgl';

import { computeLayout } from "./layout";

import { Header } from "./components/Header";
import { Body } from "./components/Body";
import { Modals } from "./components/Modals";

import { 
    GetRosServiceSingleton,
    GetApplicationServiceSingleton ,
    GetEvDScriptServiceSingleton,
    GetPendingServiceSingleton,
    GetUnityServiceSingleton
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


export class App extends Component {

    constructor(props) {
        super(props);

        // Remove these one you have stores!
        // The services are broken - better to check with Andy and Curt
        // Unity service will be removed with robotSceneManager
        //  Backend -- ROSService -- SceneStore
        // Note to Curt: Write that node for backend robot scene support (steal from Unity)
        // - Trajectories w/ heatmaps
        // - Andy's intuition is to do it all in frontend?
        // - Frontend signals & hook into zustand evdscript store -> scenestore
        const rosService = GetRosServiceSingleton();
        const appService =  GetApplicationServiceSingleton();
        const evdService = GetEvDScriptServiceSingleton();
        const pendingService = GetPendingServiceSingleton();
        const unityService = GetUnityServiceSingleton();

        this.state = {
            height: 0,
            width: 0,
            mounted: false,
            modalState: { 'settings': true },

            rosService: rosService,
            appService: appService,
            evdService: evdService,
            pendingService: pendingService,
            unityService: unityService,

            rosState: rosService.state,
            appState: appService.state,
            evdState: evdService.state,
            pendingState: pendingService.state,
            unityState: unityService.state,

            unitySimulator: new UnityContent(
                './simulator/Build/Build.json',
                './simulator/Build/UnityLoader.js',
                {
                    adjustOnWindowResize: true,
                }
            )
        };

        this.updateWindowDimensions = this.updateWindowDimensions.bind(this);

        this.rosStateUpdated = this.rosStateUpdated.bind(this);
        this.appStateUpdated = this.appStateUpdated.bind(this);
        this.evdStateUpdated = this.evdStateUpdated.bind(this);
        this.pendingStateUpdated = this.pendingStateUpdated.bind(this);
        this.unityStateUpdated = this.unityStateUpdated.bind(this);

        rosService.stateSetCallback = this.rosStateUpdated;
        appService.stateSetCallback = this.appStateUpdated;
        evdService.stateSetCallback = this.evdStateUpdated;
        pendingService.stateSetCallback = this.pendingStateUpdated;
        unityService.stateSetCallback = this.unityStateUpdated;
    }

    UNSAFE_componentWillMount() {
        this.updateWindowDimensions();
        console.log({width: window.innerWidth, height: window.innerHeight})
        window.addEventListener("resize", this.updateWindowDimensions);
    }

    componentDidMount() {
        this.updateWindowDimensions();
        this.setState({ mounted: true });
    }

    componentWillUnmount() {
        window.removeEventListener("resize", this.updateWindowDimensions);
        this.setState({ mounted: false });
    }

    updateWindowDimensions() {
        this.setState({
            width: window.innerWidth,
            height: window.innerHeight,
        });
    }

    rosStateUpdated(newState) {
        this.setState({
            rosState: newState
        });
    }

    appStateUpdated(newState) {
        this.setState({
            appState: newState
        });
    }

    evdStateUpdated(newState) {
        this.setState({
            evdState: newState
        });
    }

    pendingStateUpdated(newState) {
        this.setState({
            pendingState: newState
        });
    }

    unityStateUpdated(newState) {
        this.setState({
            unityState: newState
        });
    }

    render() {
        const {
            width,
            height,
            modalState,

            rosService,
            appService,
            evdService,
            pendingService,
            unityService,
            
            rosState,
            appState,
            evdState,
            pendingState,
            unityState,

            unitySimulator
        } = this.state;

        const { 
            theme, 
            frameStyles, 
            toggleTheme, 
            themeName,
            useChecklist
        } = this.props;

        const mainPadding = 10;
        const layoutObj = computeLayout(width, height, useChecklist);
        
        return (
            <ThemeContext.Provider 
                value={{
                    theme: theme,
                    frameStyles: frameStyles,
                    toggleTheme: toggleTheme,
                    themeName: themeName
                }}
            >
                {/*Above Keep theme context */}
                {/* Any context with service we want to change */}
                <RosContext.Provider
                    value={{
                        service: rosService,
                        ...rosState
                    }}
                >
                    <ApplicationContext.Provider
                        value={{
                            service: appService,
                            ...appState 
                        }}
                    >
                        <EvDScriptContext.Provider
                            value={{
                                service: evdService,
                                ...evdState
                            }}
                        >
                            <PendingContext.Provider 
                                value={{
                                    service: pendingService,
                                    ...pendingState
                                }}
                            >
                                <UnityContext.Provider 
                                    value={{
                                        service: unityService,
                                        ...unityState,
                                        simulator: unitySimulator
                                    }}
                                >
                                    <ModalContext.Provider 
                                        value={{
                                            openModal: (name) => {
                                                console.log('Open Modal',name);
                                                this.setState(prev => {
                                                    const newState = { ...prev.modalState};
                                                    newState[name] = true;
                                                    return {modalState: newState};
                                                })
                                            },
                                            closeModal: (name) => {
                                                console.log('Close modal',name);
                                                this.setState(prev => {
                                                    const newState = { ...prev.modalState};
                                                    newState[name] = false;
                                                    return {modalState: newState};
                                                })
                                            },
                                            state: modalState
                                        }}
                                    >
                                        
                                        <Header
                                            width={layoutObj.header.width}
                                            height={layoutObj.header.height}
                                        />
                                        
                                        <Body 
                                            layoutObj={layoutObj} 
                                            mainPadding={mainPadding} 
                                        />

                                        <Modals
                                            totalWidth={layoutObj.body.width}
                                        />          

                                    </ModalContext.Provider> 
                                </UnityContext.Provider>
                            </PendingContext.Provider>
                        </EvDScriptContext.Provider>
                    </ApplicationContext.Provider>
                </RosContext.Provider>        
            </ThemeContext.Provider>                
        );
    }
}
