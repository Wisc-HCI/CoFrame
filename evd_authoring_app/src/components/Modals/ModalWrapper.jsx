import React, { useContext } from 'react';

import { Separator } from '@fluentui/react/lib/Separator';
import { Modal } from '@fluentui/react';
import { IconButton } from '@fluentui/react/lib/Button';

import { 
    ModalContext, 
    ThemeContext 
} from '../../contexts';


export const ModalWrapper = (props) => {

    const {
        name,
        title, 
        children, 
        totalWidth,
        closeCb
    } = props;

    const width = totalWidth / 2;

    const themeContext = useContext(ThemeContext);
    const modalContext = useContext(ModalContext);

    const closeFnt = () => {
        if (closeCb) {
            closeCb();
        } else {
            modalContext.closeModal(name);
        }
    }

    return (
        <Modal
            isOpen={modalContext.state[name]}
            onDismiss={closeFnt}
            isBlocking
            styles={{ 
                main: { 
                    width: `${width}px` 
                } 
            }}
        >
            <div 
                style={{
                    borderTop: `4px solid ${themeContext.theme.palette.themePrimary}`,
                    color: themeContext.theme.palette.neutralPrimary,
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
                        rootHovered: { color: themeContext.theme.palette.neutralDark },
                    }}
                    iconProps={{ iconName: 'Cancel' }}
                    onClick={closeFnt}
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
    );
}
