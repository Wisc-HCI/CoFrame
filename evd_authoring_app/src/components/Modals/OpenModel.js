import React, { Component } from 'react';

import { Modal, IconButton } from 'office-ui-fabric-react';

import { LoremIpsum } from 'react-lorem-ipsum';

class OpenModal extends Component {
  constructor(props) {
    super(props);

    this.hideModal = this.hideModal.bind(this);
  }

  hideModal() {
    this.props.closeModal('open');
  }

  render() {
    const { open } = this.props;

    return (
      <Modal isOpen={open} onDismiss={this.hideModal} isBlocking>
        <IconButton
          styles={{
            root: { marginLeft: 'auto', marginTop: '4px', marginRight: '2px' },
          }}
          iconProps={{ iconName: 'Cancel' }}
          ariaLabel="Close popup modal"
          onClick={this.hideModal}
        />
        <LoremIpsum p={2} />
      </Modal>
    );
  }
}

export default OpenModal;
