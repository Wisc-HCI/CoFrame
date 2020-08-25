import React, { Component } from 'react'

import Logo from '../logo.svg'

import { Stack } from 'office-ui-fabric-react';
import { CommandBar } from 'office-ui-fabric-react/lib/CommandBar';

const overflowProps = { ariaLabel: 'More commands' };


class Header extends Component {

    constructor(props) {
        super(props);

        this.onUpload = this.onUpload.bind(this);
        this.onDownload = this.onDownload.bind(this);
        this.onOpen = this.onOpen.bind(this);
        this.onSettings = this.onSettings.bind(this);
    }

    static getDesiredHeight() {
        return 110; //px
    }

    static getDesiredWidth() {
        return null;
    }

    render() {

        const _items = [
            {
              key: 'newItem',
              text: 'New',
              iconProps: { iconName: 'New' },
              onClick: () => console.log('New'),
            },
            {
              key: 'upload',
              text: 'Upload',
              iconProps: { iconName: 'Upload' },
              onClick: () => console.log('Upload'),
            },
            {
              key: 'share',
              text: 'Share',
              iconProps: { iconName: 'Share' },
              onClick: () => console.log('Share'),
            },
            {
              key: 'download',
              text: 'Download',
              iconProps: { iconName: 'Download' },
              onClick: () => console.log('Download'),
            },
          ];
          
          const _overflowItems = [
            { 
                key: 'settings', 
                text: 'Settings', 
                onClick: () => console.log('Settings'), 
                iconProps: { iconName: 'Settings' } 
            },
          ];    

        return (
            <header style={{
                    width: this.props.width, 
                    height: this.props.height, 
                    overflow: 'hidden', 
                    backgroundColor: this.props.theme.semanticColors.bodyBackground, 
                    boxShadow: "3px 3px 3px #000"
                }}
            >
                <Stack horizontal>
                    
                    <img src={Logo} alt="EvD Logo" style={{height: '75px', paddingTop: '22px', paddingLeft: '20px'}} />
                    <div>
                        <Stack>
                            <Stack.Item align="center">
                                <h2><b><i>Expert View Dashboard - {this.props.filename}</i></b></h2>
                            </Stack.Item>
                            <Stack.Item align="center">
                                <CommandBar 
                                    items={_items}
                                    overflowItems={_overflowItems}
                                    overflowButtonProps={overflowProps}
                                    ariaLabel="Use left and right arrow keys to navigate between commands"
                                />
                            </Stack.Item>
                        </Stack>
                    </div>
                </Stack>
            </header>
        );
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