import React, { useContext } from 'react';

import { Separator } from '@fluentui/react/lib/Separator';
import { Modal } from '@fluentui/react';
import { IconButton } from '@fluentui/react/lib/Button';

import useGuiStore from '../../stores/GuiStore';


export const ModalWrapper = (props) => {

    const {
        name,
        title, 
        children,
        closeCb
    } = props;

    const {activeModal, closeModal} = useGuiStore(state=>({
        activeModal:state.activeModal,
        closeModal:state.closeModal
    }))

    const closeFnt = () => {
        if (closeCb) {
            closeCb();
        } else {
            closeModal(name);
        }
    }

    return (
        <Modal
            isOpen={activeModal !== null}
            onDismiss={closeFnt}
            isBlocking
            styles={{ 
                main: {width: `50vw`} 
            }}
        >
            <div 
                style={{
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
