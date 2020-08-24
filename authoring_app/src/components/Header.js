import React, { Component } from 'react'

import { Flex, FlexItem, Header as FluentHeader, Button, MenuButton, Divider, Menu } from '@fluentui/react-northstar';
import { MenuIcon, FilesUploadIcon, DownloadIcon, AppsIcon, SettingsIcon } from '@fluentui/react-icons-northstar';

class Header extends Component {

    constructor(props) {
        super(props);

        this.state = {
            menuOpen: false
        };

        this.setOpen.bind(this);
        this.onUpload.bind(this);
        this.onDownload.bind(this);
        this.onOpen.bind(this);
        this.onSettings.bind(this);
    }

    static getDesiredHeight() {
        return 85; //px
    }

    static getDesiredWidth() {
        return null;
    }

    render() {

        let MenuOptions = [
            {
                icon: (
                    <SettingsIcon />
                ),
                key: 'settings',
                content: 'Settings',
                onClick: this.onSettings
            }
        ];

        if (this.props.menuType === "MAIN") {
            MenuOptions = [
                {
                    icon: (
                        <FilesUploadIcon />
                    ),
                    key: 'upload',
                    content: 'Upload',
                    onClick: this.onUpload
                },
                {
                    icon: (
                        <DownloadIcon />
                    ),
                    key: "download",
                    content: 'Download',
                    onClick: this.onDownload
                },
                {
                    icon: (
                        <AppsIcon />
                    ),
                    key: 'open',
                    content: 'Open',
                    onClick: this.onOpen
                },
                ...MenuOptions
            ];
        }

        return (
            <header>
                <div style={{width: this.props.width, height: this.props.height}}>
                    <Flex gap="gap.small" hAlign="center" vAlign="center" styles={{paddingLeft: '10px', paddingRight: '10px', paddingTop: '5px', paddingBottom: '2px'}}>
                        <FluentHeader as="h2" content="Cobots - Expert View Dashboard" />
                        <FlexItem push>
                            <MenuButton 
                                open={this.state.menuOpen}
                                onOpenChange={(e, { open }) => this.setOpen(this.state.menuOpen)}
                                trigger={<Button icon={<MenuIcon />} title="Open Menu" />} 
                                menu={<Menu items={MenuOptions} vertical pointing="start" />}
                            />
                        </FlexItem>
                    </Flex>
                    <Divider size={2} color='brand'/>
                </div>
            </header>
        )
    }

    setOpen(open) {
        let newState = {menuOpen: !open}
        this.setState(newState);
    }

    onUpload(e) {
        console.log("On upload");
    }

    onDownload(e) {
        console.log("On download");
    }

    onOpen(e) {
        console.log("On open");
    }

    onSettings(e) {
        console.log("On Settings");
    }
}

export default Header;