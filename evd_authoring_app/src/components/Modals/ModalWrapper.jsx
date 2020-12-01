import React, { Component } from 'react';

import { Modal, IconButton, Separator } from 'office-ui-fabric-react';

class ModalWrapper extends Component {
  static generateHeaderStyle(theme) {
    return {
      borderTop: `4px solid ${theme.palette.themePrimary}`,
      color: theme.palette.neutralPrimary,
      display: 'flex',
      alignItems: 'center',
      padding: '12px 12px 14px 24px',
    };
  }

  static generateBodyStyle() {
    return {
      padding: '0 24px 24px 24px',
      overflowY: 'hidden',
      selectors: {
        p: { margin: '14px 0' },
        'p:first-child': { marginTop: 0 },
        'p:last-child': { marginBottom: 0 },
      },
    };
  }

  render() {
    const { open, title, content, theme, hideModal, width } = this.props;

    return (
      <Modal
        isOpen={open}
        onDismiss={hideModal}
        isBlocking
        styles={{ main: { width: `${width}px` } }}
      >
        <div style={ModalWrapper.generateHeaderStyle(theme)}>
          <h2>{title}</h2>
          <IconButton
            styles={{
              root: {
                marginLeft: 'auto',
                marginTop: '4px',
                marginRight: '2px',
              },
              rootHovered: { color: theme.palette.neutralDark },
            }}
            iconProps={{ iconName: 'Cancel' }}
            ariaLabel="Close popup modal"
            onClick={hideModal}
          />
        </div>
        <Separator />
        <div style={ModalWrapper.generateBodyStyle()}>{content}</div>
      </Modal>
    );
  }
}

export default ModalWrapper;
