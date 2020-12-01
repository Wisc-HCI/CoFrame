import React, { Component } from 'react';

import { PrimaryButton, DefaultButton, Stack } from 'office-ui-fabric-react';

import ModalWrapper from './ModalWrapper';

class OpenModal extends Component {
  constructor(props) {
    super(props);

    this.state = {
      selectedOption: null,
    }

    this.hideModal = this.hideModal.bind(this);
    this.submitClicked = this.submitClicked.bind(this);
  }

  hideModal() {
    const { closeModal } = this.props;
    closeModal('open');
  }

  submitClicked() {
    // TODO submit selected file action

    this.hideModal();
  }

  // eslint-disable-next-line class-methods-use-this
  generateContent() {
    const { selectedOption } = this.state;

    let submitButton;
    if (selectedOption !== null) {
      submitButton = (
        <PrimaryButton text="Submit" onClick={this.submitClicked} />
      );
    }

    return (
      <div>
        <Stack>
          <br />
          <Stack.Item align="center">
            <p>Open options menu coming soon!</p>
          </Stack.Item>
          <br />

          <Stack.Item align="center">
            <Stack horizontal tokens={{ childrenGap: '10' }}>
              {submitButton}
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
        title="Open"
        content={this.generateContent()}
        theme={theme}
        hideModal={this.hideModal}
        width={width}
      />
    );
  }
}

export default OpenModal;
