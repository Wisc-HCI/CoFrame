import React from 'react';

import { Modal, Input, Alert, InputNumber } from 'antd';

import useStore from '../../stores/Store';
import shallow from 'zustand/shallow';

export const SettingsModal = (_) => {

    const {activeModal, closeModal, url, setUrl, connection, connect, issueSettings, updateIssueSetting} = useStore(state=>({
        activeModal:state.activeModal,
        closeModal:state.closeModal,
        url:state.url,
        setUrl:state.setUrl,
        connection:state.connection,
        connect:state.connect,
        issueSettings: state.issueSettings,
        updateIssueSetting: state.updateIssueSetting
    }),shallow);
    
    let connectButtonText = 'Connect';
    if (connection === 'connecting') {
        connectButtonText = 'Connecting'
    } else if (connection === 'connected') {
        connectButtonText = 'Reconnect'
    }

    const updateIssue = (value, issue) => {
        const item = {uuid: issue.uuid, name: issue.name, value: value};
        updateIssueSetting(item);
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

            {issueSettings && Object.values(issueSettings).map((entry) => {
                return <div key={entry.name.concat('div')} style={{marginTop: 5}}>
                    <label key={entry.name.concat('label')}> {entry.name}:
                    {!entry.max && <InputNumber 
                        key={entry.name.concat('input')}
                        min={entry.min}
                        defaultValue={entry.value}
                        onChange={(e)=>updateIssue(e, entry)}/>
                    }
                    {entry.max && <InputNumber 
                        key={entry.name.concat('input')}
                        min={entry.min}
                        max={entry.max}
                        defaultValue={entry.value}
                        onChange={(e)=>updateIssue(e, entry)}/>
                    }
                    </label>
                    </div>
            })}
        </Modal>
    );
}
