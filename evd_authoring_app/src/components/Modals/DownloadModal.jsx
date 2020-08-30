import React, { Component } from 'react';

import { LoremIpsum } from 'react-lorem-ipsum';

import ModalWrapper from './ModalWrapper';

class DownloadModal extends Component {
  constructor(props) {
    super(props);

    this.hideModal = this.hideModal.bind(this);
  }

  hideModal() {
    this.props.closeModal('download');
  }

  render() {
    const { open, theme } = this.props;

    return (
      <ModalWrapper
        open={open}
        title="Download"
        content={<LoremIpsum p={1} />}
        theme={theme}
        hideModal={this.hideModal}
      />
    );
  }
}

export default DownloadModal;
