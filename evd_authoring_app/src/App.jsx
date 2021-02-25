import React from "react";

import { computeLayout } from "./layout";

import { Header } from "./components/Header";
import { Body } from "./components/Body";

import { SettingsModal } from "./components/SettingsModal";
import { UploadModal } from "./components/UploadModal";
import { DownloadModal } from "./components/DownloadModal";
import { OpenModal } from "./components/OpenModel";

export class App extends React.Component {

    constructor(props) {
        super(props);

        this.state = {
            height: 0,
            width: 0,

            downloadModalOpen: false,
            uploadModalOpen: false,
            openModalOpen: false,
            settingsModalOpen: false,

            filename: "Untitled",
            model: null
        };

        this.updateWindowDimensions = this.updateWindowDimensions.bind(this);
        this.closeModal = this.closeModal.bind(this);
        this.onHeaderButtonClicked = this.onHeaderButtonClicked.bind(this);
    }

    UNSAFE_componentWillMount() {
        this.updateWindowDimensions();
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

    closeModal(modal) {
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
            settingsModalOpen,
            filename,
        } = this.state;

        const { theme } = this.props;

        const mainPadding = 10;
        const layoutObj = computeLayout(width, height);
        
        return (
            <React.Fragment>
                <Header
                    theme={theme}
                    width={layoutObj.headerWidth}
                    height={layoutObj.headerHeight}
                    filename={filename}
                    onButtonClick={this.onHeaderButtonClicked}
                />

                <Body 
                    layoutObj={layoutObj} 
                    theme={theme} 
                    mainPadding={mainPadding} 
                />

                <DownloadModal
                    open={downloadModalOpen}
                    closeModal={this.closeModal}
                    theme={theme}
                    totalWidth={layoutObj.totalWidth}
                />
                <UploadModal
                    open={uploadModalOpen}
                    closeModal={this.closeModal}
                    theme={theme}
                    totalWidth={layoutObj.totalWidth}
                />
                <OpenModal
                    open={openModalOpen}
                    closeModal={this.closeModal}
                    theme={theme}
                    totalWidth={layoutObj.totalWidth}
                />
                <SettingsModal
                    open={settingsModalOpen}
                    closeModal={this.closeModal}
                    theme={theme}
                    totalWidth={layoutObj.totalWidth}
                />
            </React.Fragment>
        );
    }
}
