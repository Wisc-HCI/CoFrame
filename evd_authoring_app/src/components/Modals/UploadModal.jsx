import React, { useState } from 'react';
import YAML from 'yaml';

import { Modal, Upload, Form, Button, Input, Result, message } from 'antd';
import { UploadOutlined } from '@ant-design/icons';

import useApplicationStore from '../../stores/ApplicationStore';
import useGuiStore from '../../stores/GuiStore';

export const UploadModal = (_) => {

    const {activeModal, closeModal} = useGuiStore(state=>({
        activeModal:state.activeModal,
        closeModal:state.closeModal
    }));
    const {filename, setFilename} = useApplicationStore(state=>({
        filename:state.filename,
        setFilename:state.setFilename
    }))


    const [data, setData] = useState(null);

    const handleUpload = async (info) => {
        if (info.file) {
          const reader = new FileReader();
          reader.onabort = () => {message.error('Upload Aborted')}
          reader.onerror = () => {message.error('Upload Error')}
          reader.onload = () => {
    
            let data = YAML.parse(reader.result);
            if (data) {
              // Do handling
              setData(data)
              closeModal()
            }
          }
          reader.readAsText(info.file.originFileObj)
    
        }
    }

    return (
        <Modal title="Upload" visible={activeModal === 'upload'} onOk={closeModal} onCancel={closeModal}>
            <Upload.Dragger name='file' onChange={(info)=>handleUpload(info)} accept='text/yaml, text/json'>
              <p className="ant-upload-drag-icon">
                <UploadOutlined/>
              </p>
              <p className="ant-upload-text">Click or drag file to this area to upload</p>
              <p className="ant-upload-hint">
                Upload a Project File
              </p>
            </Upload.Dragger>
        </Modal>
    );
}