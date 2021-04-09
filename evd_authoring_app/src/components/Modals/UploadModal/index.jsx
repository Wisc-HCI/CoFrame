import React, { useContext, useState } from 'react';

import {
    TextField,
    PrimaryButton,
    Stack,
    Spinner,
} from 'office-ui-fabric-react';

import { MetaData } from './MetaData';
import { ModalWrapper } from '../ModalWrapper';
import { ModalControlButtons } from '../ModalControlButtons';

import { 
    ApplicationContext, 
    ModalContext 
} from '../../../contexts';

export const UploadModal = (props) => {

    const { totalWidth } = props;

    const [filename, setFilename] = useState('');
    const [pending, setPending] = useState(false);
    const [data, setData] = useState(null);
    const [parsed, setParsed] = useState(false);

    const modalContext = useContext(ModalContext);
    const appContext = useContext(ApplicationContext);

    let metaData = null;
    if (pending) {
        metaData = (<Spinner label="Uploading" />);
    } else if (data !== null) {
        if (parsed) {
            metaData = (<MetaData data={data} />);

        } else {
            metaData = (
                <Stack.Item align="center">
                    <p>Failed to parse file</p>
                    <br />
                    <p>Please upload a valid EvD JSON file</p>
                </Stack.Item>
            );
        }
    } else {
        metaData = (
            <Stack.Item align="center">
                <p>Please upload JSON file</p>
            </Stack.Item>
        );
    }

    const closeFnt = () => {
        setFilename('');
        setPending(false);
        setData(null);
        setParsed(false);

        modalContext.closeModal('upload');
    };

    return (
        <ModalWrapper
            name="upload"
            title="Upload"
            totalWidth={totalWidth}
            closeCb={closeFnt}
        >
            <input
                type="file"
                id="fileupload"
                style={{ display: 'none' }}
                onChange={(event) => {
                    const { files } = event.target;
                    const reader = new FileReader();
            
                    reader.addEventListener('load', (e) => {
                        const isOpen = modalContext.state['upload'];
                        if (isOpen) {
            
                            let d = null;
                            let success = null;
            
                            try {
                                d = JSON.parse(e.target.result);
                                success = true;
                            } catch (ex) {
                                success = false;
                            }

                            setPending(false);
                            setData(d);
                            setParsed(success);
                        }
                    });
            
                    reader.readAsText(files[0]);

                    setPending(true);
                    setData(null);
                    setFilename(files[0].name);
                }}
                accept=".json, application/json, application/JSON"
            />

            <Stack>
                <Stack horizontal tokens={{ childrenGap: '2' }}>

                    <Stack.Item>
                        <PrimaryButton
                            text="File"
                            iconProps={{ iconName: 'Upload' }}
                            allowDisabledFocus
                            onClick={() => {
                                document.getElementById('fileupload').click();
                            }}
                        />
                    </Stack.Item>

                    <Stack.Item grow>
                        <TextField label="" readOnly value={filename} />
                    </Stack.Item>
                </Stack>

                <br />
                {metaData}
                <br />

                <Stack.Item align="center">
                    <ModalControlButtons 
                        order={['submit','cancel']} 
                        callbacks={{
                            'submit': () => {
                                const niceFilename = filename.split('.').slice(0, -1).join('.');
                                appContext.service.loadFromFile(data, niceFilename);
                                closeFnt();
                            }, 
                            'cancel': closeFnt
                        }}
                        isPrimary={{
                            'submit': true, 
                            'cancel': false
                        }}
                        disabled={{'submit': !parsed}}
                    />   
                </Stack.Item>
            </Stack>
        </ModalWrapper>
    );
}