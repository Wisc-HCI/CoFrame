import React, { Component } from 'react';

import {
  TextField,
  PrimaryButton,
  DefaultButton,
  Stack,
  Spinner,
} from 'office-ui-fabric-react';

import ModalWrapper from './ModalWrapper';

class UploadModal extends Component {
  constructor(props) {
    super(props);

    this.state = {
      filename: '',
      pending: false,
      data: null,
      fileSuccessfullyParsed: false,
    };

    this.hideModal = this.hideModal.bind(this);
    this.uploadClicked = this.uploadClicked.bind(this);
    this.submitClicked = this.submitClicked.bind(this);
    this.fileChanged = this.fileChanged.bind(this);
  }

  hideModal() {
    const { closeModal } = this.props;
    closeModal('upload');

    this.setState((prevState) => {
      return {
        ...prevState,
        filename: '',
        data: null,
        fileSuccessfullyParsed: false,
        pending: false,
      };
    });
  }

  // eslint-disable-next-line class-methods-use-this
  uploadClicked() {
    const fileElement = document.getElementById('fileupload');
    fileElement.click();
  }

  fileChanged(event) {
    const { files } = event.target;
    const reader = new FileReader();

    reader.addEventListener('load', (e) => {
      const { open } = this.props;

      if (open) {
        let data = null;
        let success = null;
        try {
          data = JSON.parse(e.target.result);
          success = true;
        } catch (ex) {
          success = false;
        }

        this.setState((prevState) => {
          return {
            ...prevState,
            pending: false,
            data,
            fileSuccessfullyParsed: success,
          };
        });
      }
    });

    reader.readAsText(files[0]);

    this.setState((prevState) => {
      return {
        ...prevState,
        pending: true,
        data: null,
        filename: files[0].name,
      };
    });
  }

  submitClicked() {
    // TODO submit data action
    console.log('Submit Clicked');

    this.hideModal();
  }

  generateContent() {
    const { filename, fileSuccessfullyParsed, pending, data } = this.state;

    let metaData = null;
    if (pending) {
      metaData = (
        <div>
          <Spinner label="Uploading" ariaLive="assertive" />
        </div>
      );
    } else if (data !== null) {
      if (fileSuccessfullyParsed) {
        metaData = (
          <div>
            <p>META DATA : Work in progress</p>
          </div>
        );
      } else {
        metaData = (
          <Stack.Item align="center">
            <p>Failed to parse file</p>
          </Stack.Item>
        );
      }
    } else {
      metaData = (
        <Stack.Item align="center">
          <p>Please upload file</p>
        </Stack.Item>
      );
    }

    return (
      <div>
        <input
          type="file"
          id="fileupload"
          style={{ display: 'none' }}
          onChange={this.fileChanged}
          accept=".json, application/json, application/JSON"
        />

        <Stack>
          <Stack horizontal tokens={{ childrenGap: '2' }}>
            <Stack.Item>
              <PrimaryButton
                text="File"
                iconProps={{ iconName: 'Upload' }}
                allowDisabledFocus
                onClick={this.uploadClicked}
              />
            </Stack.Item>

            <Stack.Item grow>
              <TextField label="" readOnly value={filename} />
            </Stack.Item>
          </Stack>

          <br />
          {metaData}
          <br />

          <Stack.Item align="center">
            <Stack horizontal tokens={{ childrenGap: '10' }}>
              <PrimaryButton
                text="Submit"
                onClick={this.submitClicked}
                disabled={!fileSuccessfullyParsed}
              />
              <DefaultButton text="Cancel" onClick={this.hideModal} />
            </Stack>
          </Stack.Item>
        </Stack>
      </div>
    );
  }

  render() {
    const { open, theme, totalWidth } = this.props;

    const width = totalWidth / 2;

    return (
      <ModalWrapper
        open={open}
        title="Upload"
        content={this.generateContent()}
        theme={theme}
        hideModal={this.hideModal}
        width={width}
      />
    );
  }
}

export default UploadModal;
