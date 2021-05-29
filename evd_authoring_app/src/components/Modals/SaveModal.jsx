import React from 'react';

import { Modal, Input } from 'antd';

import useApplicationStore from '../../stores/ApplicationStore';
import useGuiStore from '../../stores/GuiStore';


export const SaveModal = (_) => {

    const {activeModal, closeModal} = useGuiStore(state=>({
        activeModal:state.activeModal,
        closeModal:state.closeModal
    }));
    const {save, filename, setFilename} = useApplicationStore(state=>({
        save:state.save,
        filename:state.filename,
        setFilename:state.setFilename,
    }));

    return (
        <Modal title="Save As" visible={activeModal === 'save'} onOk={closeModal} onCancel={closeModal}>
            <Input.Search 
                placeholder="e.g. My-First-Project" 
                size="large" 
                defaultValue={filename} 
                enterButton='Save'
                onSearch={setFilename}
            />
        </Modal>
    );
};