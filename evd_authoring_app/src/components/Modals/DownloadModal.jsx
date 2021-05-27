import React, { useState, useContext }  from 'react';

import { saveAs } from 'file-saver';

import { Modal, Result, Spin } from 'antd';
import { LoadingOutlined } from '@ant-design/icons';

import useApplicationStore from '../../stores/ApplicationStore';
import useGuiStore from '../../stores/GuiStore';
import useEvdStore from '../../stores/EvdStore';


export const DownloadModal = (_) => {

    const [downloading, setDownloading] = useState(false);

    const {activeModal, closeModal} = useGuiStore(state=>({
        activeModal:state.activeModal,
        closeModal:state.closeModal
    }));
    
    const filename = useApplicationStore(state=>state.filename);
    const evdProgram = useEvdStore(state=>state.program);
    


    if (activeModal === 'download' && !downloading) {

        const program = evdProgram;
        const text = JSON.stringify((program === null) ? {} : program.toDict());

        const blob = new Blob([text], { type: 'text/plain;charset=utf-8' });
        saveAs(blob, `${filename}.json`);
        setDownloading(true);
    }

    return (
        <Modal title="Download" visible={activeModal === 'download'} onOk={closeModal} onCancel={closeModal}>
            <Result
                title={<p>File is downloading. Press the <i><b>Close</b></i> button to return to design.</p>}
                icon={<Spin indicator={<LoadingOutlined style={{ fontSize: 24 }} spin />}/>}
            />,
        </Modal>
    );
}