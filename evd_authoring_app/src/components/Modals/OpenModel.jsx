import React, { Component } from 'react';

import { LoremIpsum } from 'react-lorem-ipsum';

import ModalWrapper from './ModalWrapper';

class OpenModal extends Component {
  constructor(props) {
    super(props);

    this.hideModal = this.hideModal.bind(this);
  }

  hideModal() {
    this.props.closeModal('open');
  }

  render() {
    const { open, theme } = this.props;

    return (
      <ModalWrapper
        open={open}
        title="Open"
        content={<LoremIpsum p={2} />}
        theme={theme}
        hideModal={this.hideModal}
      />
    );
  }
}

export default OpenModal;
