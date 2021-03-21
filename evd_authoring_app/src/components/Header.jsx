import React from 'react';

import { Stack } from 'office-ui-fabric-react';
import { CommandBar } from 'office-ui-fabric-react/lib/CommandBar';

import Logo from '../content/logo.svg';

import { 
    ThemeContext,
    ApplicationContext
} from "../contexts";



export const Header = (props) => {

    const { 
        width, 
        height, 
        onButtonClick 
    } = props;

    const items = [
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
        <ThemeContext.Consumer>
            { value => (
                <header
                    style={{
                        width: `${width}px`,
                        height: `${height}px`,
                        backgroundColor: value.theme.semanticColors.bodyBackground,
                        boxShadow: '3px 3px 3px #000'
                    }}
                >
                    <Stack horizontal>
                        <img
                            src={Logo}
                            alt="EvD Logo"
                            style={{
                                height: '40px',
                                paddingTop: '13px',
                                paddingLeft: '13px',
                                paddingRight: '13px'
                            }}
                        />

                        <Stack.Item align="start">
                            <ApplicationContext.Consumer>
                                { appValue => (
                                    <h2><b><i>Expert View Dashboard - {appValue.filename}</i></b></h2>
                                )}
                            </ApplicationContext.Consumer>
                        </Stack.Item>

                        <Stack.Item grow align="center">
                            <CommandBar
                                items={items.map(entry => {
                                    return {
                                        key: entry.key,
                                        text: entry.name,
                                        iconProps: { iconName: entry.iconName },
                                        onClick: () => {onButtonClick(entry.key)}
                                    };
                                })}
                            />
                        </Stack.Item>
                    </Stack>
                </header>
            )}
        </ThemeContext.Consumer>
    );
}
