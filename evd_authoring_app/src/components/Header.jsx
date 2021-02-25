import React, { Component } from 'react';

import { Stack } from 'office-ui-fabric-react';
import { CommandBar } from 'office-ui-fabric-react/lib/CommandBar';

import Logo from '../content/logo.svg';

export class Header extends Component {
  constructor(props) {
    super(props);

    this.onUpload = this.onUpload.bind(this);
    this.onDownload = this.onDownload.bind(this);
    this.onOpen = this.onOpen.bind(this);
    this.onSettings = this.onSettings.bind(this);
    this.onSave = this.onSave.bind(this);
  }

  onUpload() {
    const { onButtonClick } = this.props;
    console.log('On upload');
    onButtonClick('upload');
  }

  onDownload() {
    const { onButtonClick } = this.props;
    console.log('On download');
    onButtonClick('download');
  }

  onOpen() {
    const { onButtonClick } = this.props;
    console.log('On open');
    onButtonClick('open');
  }

  onSettings() {
    const { onButtonClick } = this.props;
    console.log('On Settings');
    onButtonClick('settings');
  }

  onSave() {
      const { onButtonClick } = this.props;
      console.log('On Save');
      onButtonClick('save');
  }

  createButtonList() {
    return [
      {
        key: 'open',
        text: 'Open',
        iconProps: { iconName: 'OpenFolderHorizontal' },
        onClick: this.onOpen,
      },
      {
          key: 'save',
          text: 'Save',
          iconProps: { iconName: 'Save' },
          onClick: this.onSave,
      },
      {
        key: 'upload',
        text: 'Upload',
        iconProps: { iconName: 'Upload' },
        onClick: this.onUpload,
      },
      {
        key: 'download',
        text: 'Download',
        iconProps: { iconName: 'Download' },
        onClick: this.onDownload,
      },
      {
        key: 'settings',
        text: 'Settings',
        onClick: this.onSettings,
        iconProps: { iconName: 'Settings' },
      },
    ];
  }

  render() {
    const { width, height, theme, filename } = this.props;

    const items = this.createButtonList();

    return (
      <header
        style={{
          width: `${width}px`,
          height: `${height}px`,
          backgroundColor: theme.semanticColors.bodyBackground,
          boxShadow: '3px 3px 3px #000',
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
              paddingRight: '13px',
            }}
          />

          <Stack.Item align="start">
            <h2>
              <b>
                <i>Expert View Dashboard - {filename}</i>
              </b>
            </h2>
          </Stack.Item>

          <Stack.Item grow align="center">
            <CommandBar
              items={items}
              ariaLabel="Use left and right arrow keys to navigate between commands"
            />
          </Stack.Item>
        </Stack>
      </header>
    );
  }
}
