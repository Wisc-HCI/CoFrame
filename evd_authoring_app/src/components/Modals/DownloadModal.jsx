import React, { Component } from 'react';

import { saveAs } from 'file-saver';
import { Stack, PrimaryButton } from 'office-ui-fabric-react';

import ModalWrapper from './ModalWrapper';

class DownloadModal extends Component {
  constructor(props) {
    super(props);

    this.state = {
      downloading: false,
      text: '{}',
      filename: 'untitled.json',
    };

    this.hideModal = this.hideModal.bind(this);
  }

  hideModal() {
    this.setState((prevState) => {
      return {
        ...prevState,
        downloading: false,
      };
    });

    const { closeModal } = this.props;
    closeModal('download');
  }

  // eslint-disable-next-line class-methods-use-this
  generateContent() {
    return (
      <Stack>
        <br />
        <Stack.Item align="center">
          <p>
            File is downloading. Press the{' '}
            <i>
              <b>Close</b>
            </i>{' '}
            button to return to design.
          </p>
        </Stack.Item>
        <br />

        <Stack.Item align="center">
          <PrimaryButton text="Close" onClick={this.hideModal} />
        </Stack.Item>
      </Stack>
    );
  }

  render() {
    const { open, theme, totalWidth } = this.props;
    const { downloading, text, filename } = this.state;

    const width = totalWidth / 2;

    if (open && !downloading) {
      const blob = new Blob([text], { type: 'text/plain;charset=utf-8' });
      saveAs(blob, filename);

      this.setState((prevState) => {
        return {
          ...prevState,
          downloading: true,
        };
      });
    }

    return (
      <ModalWrapper
        open={open}
        title="Download"
        content={this.generateContent()}
        theme={theme}
        hideModal={this.hideModal}
        width={width}
      />
    );
  }
}

export default DownloadModal;
