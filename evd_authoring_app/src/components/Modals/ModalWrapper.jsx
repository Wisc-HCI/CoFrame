import React from 'react';

import { 
    Modal, 
    IconButton, 
    Separator 
} from 'office-ui-fabric-react';

import { ThemeContext } from '../../contexts';


export const ModalWrapper = (props) => {

    const { 
        open, 
        title, 
        children, 
        hideModal, 
        width 
    } = props;

    return (
        <ThemeContext.Consumer>
            { value => (
                <Modal
                    isOpen={open}
                    onDismiss={hideModal}
                    isBlocking
                    styles={{ 
                        main: { 
                            width: `${width}px` 
                        } 
                    }}
                >
                    <div 
                        style={{
                            borderTop: `4px solid ${value.theme.palette.themePrimary}`,
                            color: value.theme.palette.neutralPrimary,
                            display: 'flex',
                            alignItems: 'center',
                            padding: '12px 12px 14px 24px',
                        }}
                    >
                        <h2>{title}</h2>
                        
                        <IconButton
                            styles={{
                                root: {
                                    marginLeft: 'auto',
                                    marginTop: '4px',
                                    marginRight: '2px',
                                },
                                rootHovered: { color: value.theme.palette.neutralDark },
                            }}
                            iconProps={{ iconName: 'Cancel' }}
                            ariaLabel="Close popup modal"
                            onClick={hideModal}
                        />
                    </div>
                    
                    <Separator />
                    
                    <div 
                        style={{
                            padding: '0 24px 24px 24px',
                            overflowY: 'hidden'
                        }}
                    >
                        {children}
                    </div>
                </Modal>
            )}
        </ThemeContext.Consumer>
    );
}
