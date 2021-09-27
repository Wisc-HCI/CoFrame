import React from 'react';

import { Modal, Input, Alert } from 'antd';

import useStore from '../../stores/Store';
import shallow from 'zustand/shallow';

export const SettingsModal = (_) => {

    const {activeModal, closeModal, url, setUrl, connection, connect} = useStore(state=>({
        activeModal:state.activeModal,
        closeModal:state.closeModal,
        url:state.url,
        setUrl:state.setUrl,
        connection:state.connection,
        connect:state.connect
    }),shallow);
    
    let connectButtonText = 'Connect';
    if (connection === 'connecting') {
        connectButtonText = 'Connecting'
    } else if (connection === 'connected') {
        connectButtonText = 'Reconnect'
    }

    return (
        <Modal title="Settings" visible={activeModal === 'settings'} onOk={closeModal} onCancel={closeModal}>
            {connection === 'connected' && (
                <Alert type="success" showIcon message="Connected!"/>
            )}
            {connection === 'connecting' && (
                <Alert type="warning" showIcon message="Connecting..."/>
            )}
            {connection === 'disconnected' && (
                <Alert type="error" showIcon message="Not Connected!"/>
            )}
            
            <Input.Search 
                style={{marginTop:20}}
                placeholder="e.g. ws://localhost:9090" 
                defaultValue={url} 
                onChange={(e)=>setUrl(e.target.value)}
                enterButton={connectButtonText} 
                size="large" 
                loading={connection === 'connecting'} 
                onSearch={connect}
                disabled={connection === 'connecting'}
            />
        </Modal>
    );
}
