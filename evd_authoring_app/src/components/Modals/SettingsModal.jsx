import React, { Component } from 'react';

import { LoremIpsum } from 'react-lorem-ipsum';

import ModalWrapper from './ModalWrapper';

class SettingsModal extends Component {
  constructor(props) {
    super(props);

    this.hideModal = this.hideModal.bind(this);
  }

  hideModal() {
    this.props.closeModal('settings');
  }

  render() {
    const { open, theme } = this.props;

    return (
      <ModalWrapper
        open={open}
        title="Settings"
        content={<LoremIpsum p={3} />}
        theme={theme}
        hideModal={this.hideModal}
      />
    );
  }
}

export default SettingsModal;
