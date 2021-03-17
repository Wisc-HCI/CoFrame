import React from "react";

import { computeLayout } from "./layout";

import { Header } from "./components/Header";
import { Body } from "./components/Body";

import { SettingsModal } from "./components/Modals/SettingsModal";
import { UploadModal } from "./components/Modals/UploadModal";
import { DownloadModal } from "./components/Modals/DownloadModal";
import { OpenModal } from "./components/Modals/OpenModel";

import { ThemeContext } from "./contexts";


export class App extends React.Component {

    constructor(props) {
        super(props);

        this.state = {
            height: 0,
            width: 0,
            downloadModalOpen: false,
            uploadModalOpen: false,
            openModalOpen: false,
            settingsModalOpen: true
        };

        this.updateWindowDimensions = this.updateWindowDimensions.bind(this);
        this.closeModal = this.closeModal.bind(this);
        this.onHeaderButtonClicked = this.onHeaderButtonClicked.bind(this);
    }

    UNSAFE_componentWillMount() {
        this.updateWindowDimensions();
        console.log({width: window.innerWidth, height: window.innerHeight})
        window.addEventListener("resize", this.updateWindowDimensions);
    }

    componentDidMount() {
        this.updateWindowDimensions();
    }

    componentWillUnmount() {
        window.removeEventListener("resize", this.updateWindowDimensions);
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

    render() {
        const {
            width,
            height,
            downloadModalOpen,
            uploadModalOpen,
            openModalOpen,
            settingsModalOpen
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
                
            </ThemeContext.Provider>                
        );
    }
}
