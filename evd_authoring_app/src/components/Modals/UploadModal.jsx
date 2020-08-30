import React, { Component } from 'react';

import { LoremIpsum } from 'react-lorem-ipsum';

import ModalWrapper from './ModalWrapper';

class UploadModal extends Component {
  constructor(props) {
    super(props);

    this.hideModal = this.hideModal.bind(this);
  }

  hideModal() {
    this.props.closeModal('upload');
  }

  render() {
    const { open, theme } = this.props;

    return (
      <ModalWrapper
        open={open}
        title="Upload"
        content={<LoremIpsum p={4} />}
        theme={theme}
        hideModal={this.hideModal}
      />
    );
  }
}

export default UploadModal;
