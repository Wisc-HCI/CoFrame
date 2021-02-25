import React, { Component } from 'react';

import { PrimaryButton, DefaultButton, Stack } from 'office-ui-fabric-react';

import { ModalWrapper } from './ModalWrapper';

export class SettingsModal extends Component {
  constructor(props) {
    super(props);

    this.state = {
      changesMade: false,
    };

    this.hideModal = this.hideModal.bind(this);
    this.saveClicked = this.saveClicked.bind(this);
  }

  hideModal() {
    const { closeModal } = this.props;
    closeModal('settings');
  }

  saveClicked() {
    // TODO commit changes action

    this.hideModal();
  }

  // eslint-disable-next-line class-methods-use-this
  generateContent() {
    const { changesMade } = this.state;

    let saveButton;
    if (changesMade) {
      saveButton = (
        <PrimaryButton text="Save Changes" onClick={this.saveClicked} />
      );
    }

    return (
      <div>
        <Stack>
          <br />
          <Stack.Item align="center">
            <p>Settings coming soon!</p>
          </Stack.Item>
          <br />

          <Stack.Item align="center">
            <Stack horizontal tokens={{ childrenGap: '10' }}>
              {saveButton}
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
        title="Settings"
        content={this.generateContent()}
        theme={theme}
        hideModal={this.hideModal}
        width={width}
      />
    );
  }
}
