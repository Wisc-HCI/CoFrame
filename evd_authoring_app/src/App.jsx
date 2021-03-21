import React from "react";

import { computeLayout } from "./layout";

import { Header } from "./components/Header";
import { Body } from "./components/Body";

import { SettingsModal } from "./components/Modals/SettingsModal";
import { UploadModal } from "./components/Modals/UploadModal";
import { DownloadModal } from "./components/Modals/DownloadModal";
import { OpenModal } from "./components/Modals/OpenModel";

import { 
    GetRosServiceSingleton,
    GetApplicationServiceSingleton 
} from './services';

import { 
    ThemeContext,
    ApplicationContext,
    RosContext
} from "./contexts";


export class App extends React.Component {

    constructor(props) {
        super(props);

        const rosService = GetRosServiceSingleton();
        const appService =  GetApplicationServiceSingleton();

        this.state = {
            height: 0,
            width: 0,
            mounted: false,
            downloadModalOpen: false,
            uploadModalOpen: false,
            openModalOpen: false,
            settingsModalOpen: true,
            rosService: rosService,
            appService: appService,
            rosState: rosService.state,
            appState: appService.state
        };

        this.updateWindowDimensions = this.updateWindowDimensions.bind(this);
        this.closeModal = this.closeModal.bind(this);
        this.onHeaderButtonClicked = this.onHeaderButtonClicked.bind(this);
        this.rosStateUpdated = this.rosStateUpdated.bind(this);
        this.appStateUpdated = this.appStateUpdated.bind(this);

        rosService.stateSetCallback = this.rosStateUpdated;
        appService.stateSetCallback = this.appStateUpdated;
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

    onHeaderButtonClicked(button) {
        this.setState({
            downloadModalOpen: button === "download",
            uploadModalOpen: button === "upload",
            openModalOpen: button === "open",
            settingsModalOpen: button === "settings",
        });
    }

    closeModal() {
        this.setState({
            downloadModalOpen: false,
            uploadModalOpen: false,
            openModalOpen: false,
            settingsModalOpen: false,
        });
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

    render() {
        const {
            width,
            height,
            downloadModalOpen,
            uploadModalOpen,
            openModalOpen,
            settingsModalOpen,
            rosService,
            appService,
            rosState,
            appState
        } = this.state;

        const { 
            theme, 
            frameStyles, 
            toggleTheme, 
            themeName 
        } = this.props;

        const mainPadding = 10;
        const layoutObj = computeLayout(width, height);
        
        return (
            <ThemeContext.Provider 
                value={{
                    theme: theme,
                    frameStyles: frameStyles,
                    toggleTheme: toggleTheme,
                    themeName: themeName
                }}
            >
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

                        <Header
                            filename="Untitled"
                            width={layoutObj.header.width}
                            height={layoutObj.header.height}
                            onButtonClick={this.onHeaderButtonClicked}
                        />

                        <Body 
                            layoutObj={layoutObj} 
                            mainPadding={mainPadding} 
                        />

                        <DownloadModal
                            open={downloadModalOpen}
                            closeModal={this.closeModal}
                            totalWidth={layoutObj.totalWidth}
                        />
                        <UploadModal
                            open={uploadModalOpen}
                            closeModal={this.closeModal}
                            totalWidth={layoutObj.totalWidth}
                        />
                        <OpenModal
                            open={openModalOpen}
                            closeModal={this.closeModal}
                            totalWidth={layoutObj.totalWidth}
                        />
                        <SettingsModal
                            open={settingsModalOpen}
                            closeModal={this.closeModal}
                            totalWidth={layoutObj.totalWidth}
                        />

                    </ApplicationContext.Provider>
                </RosContext.Provider>        
            </ThemeContext.Provider>                
        );
    }
}
