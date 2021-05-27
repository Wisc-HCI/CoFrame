import React from 'react';

import { Modal, Empty } from 'antd';

import useGuiStore from '../../stores/GuiStore';

export const OpenModal = (_) => {

    const {activeModal, closeModal} = useGuiStore(state=>({
        activeModal:state.activeModal,
        closeModal:state.closeModal
    }));

    return (
        <Modal title="Open" visible={activeModal === 'open'} onOk={closeModal} onCancel={closeModal}>
            <Empty/>
        </Modal>
    );
}