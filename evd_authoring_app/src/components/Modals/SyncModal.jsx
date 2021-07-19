import React, {useRef} from 'react';

import { saveAs } from 'file-saver';
import YAML from 'yaml';
import { Modal, Button, Row, Col, Upload, message } from 'antd';
import { UploadOutlined, DownloadOutlined } from '@ant-design/icons';

import useGuiStore from '../../stores/GuiStore';
import useEvdStore from '../../stores/EvdStore';


export const SyncModal = (_) => {

    const { activeModal, closeModal } = useGuiStore(state => ({
        activeModal: state.activeModal,
        closeModal: state.closeModal
    }));

    const fileInputRef = useRef();

    const [getProgram, setProgram] = useEvdStore(state => [state.getProgram, state.setProgram]);

    const download = () => {
        const program = getProgram();
        const text = JSON.stringify(program);

        const blob = new Blob([text], { type: 'text/plain;charset=utf-8' });
        saveAs(blob, `${program.name.replace(' ', '_')}.json`);
    }

    const upload = async (event) => {
        console.log(event);
        const fileUploaded = event.target.files[0];
        console.log(fileUploaded)
        if (fileUploaded) {
            const reader = new FileReader();
            reader.onabort = () => { message.error('Upload Aborted') }
            reader.onerror = () => { message.error('Upload Error') }
            reader.onload = () => {

                let data = YAML.parse(reader.result);
                if (data) {
                    // Do handling
                    setProgram(data)
                    closeModal()
                }
            }
            reader.readAsText(fileUploaded)
        }
        //
        // props.handleFile(fileUploaded);
        // console.log(file)
        // if (file) {
        //     

        // }
    }

    const handleUploadClick = (_) => {
        fileInputRef.current.click();
    };

    return (
        <Modal title="Upload and Download" visible={activeModal === 'sync'} onOk={closeModal} onCancel={closeModal}>
            <Row style={{ width: '100%' }}>
                <Col span={12} style={{ padding: 10 }}>
                    <input
                        type='file'
                        ref={fileInputRef}
                        onChange={upload}
                        style={{display: 'none'}}
                    />
                    <Button type='primary' size='large' block icon={<UploadOutlined />} onClick={handleUploadClick}>Upload</Button>
                </Col>
                <Col span={12} style={{ padding: 10 }}>
                    <Button type='primary' size='large' block icon={<DownloadOutlined />} onClick={download}>Download</Button>
                </Col>
            </Row>
        </Modal>
    );
}