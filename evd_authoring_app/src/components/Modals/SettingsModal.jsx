import React from 'react';

import { Modal, Form, Input } from 'antd';

import useApplicationStore from '../../stores/ApplicationStore';
import useGuiStore from '../../stores/GuiStore';
import useRosStore from '../../stores/RosStore';

export const SettingsModal = (_) => {

    const {activeModal, closeModal} = useGuiStore(state=>({
        activeModal:state.activeModal,
        closeModal:state.closeModal
    }));
    const {url, setUrl, connection, connect} = useRosStore(state=>({
        url:state.url,
        setUrl:state.setUrl,
        connection:state.connection,
        connect:state.connect
    }))
    const {filename, setFilename} = useApplicationStore(state=>({
        filename:state.filename,
        setFilename:state.setFilename
    }))

    let connectButtonText = 'Connect';
    if (connection === 'connecting') {
        connectButtonText = 'Connecting'
    } else if (connection === 'connected') {
        connectButtonText = 'Reconnect'
    }

    return (
        <Modal title="Settings" visible={activeModal === 'settings'} onOk={closeModal} onCancel={closeModal}>
            <Form>
                <Form.Item label="Project Name" name="project-name">
                    <Input placeholder="e.g. My-First-Project" size="large" onChange={e=>setFilename(e.target.value)} defaultValue={filename}/>
                </Form.Item>
            </Form>

            <Form>
                <Form.Item label="Server" name="server">
                    <Input.Search 
                        placeholder="e.g. ws://localhost:9090" 
                        defaultValue={url} 
                        enterButton={connectButtonText} 
                        size="large" 
                        loading={connection === 'connecting'} 
                        onSearch={connect}
                        disabled={connection === 'connecting'}
                    />
                </Form.Item>
            </Form>
        </Modal>
    );
}
