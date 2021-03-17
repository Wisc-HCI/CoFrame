import React  from 'react';

import {
    TextField,
    PrimaryButton,
    Stack,
    Spinner,
} from 'office-ui-fabric-react';

import { ModalWrapper } from './ModalWrapper';
import { ModalControlButtons } from './ModalControlButtons';


export class UploadModal extends React.Component {

    constructor(props) {
        super(props);

        this.state = {
            filename: '',
            pending: false,
            data: null,
            fileSuccessfullyParsed: false,
        };

        this.hideModal = this.hideModal.bind(this);
        this.uploadClicked = this.uploadClicked.bind(this);
        this.submitClicked = this.submitClicked.bind(this);
        this.fileChanged = this.fileChanged.bind(this);
    }

    hideModal() {
        this.setState((prevState) => {
            return {
                ...prevState,
                filename: '',
                data: null,
                fileSuccessfullyParsed: false,
                pending: false,
            };
        });

        const { closeModal } = this.props;
        closeModal('upload');
    }

    uploadClicked() {
        const fileElement = document.getElementById('fileupload');
        fileElement.click();
    }

    fileChanged(event) {
        const { files } = event.target;
        const reader = new FileReader();

        reader.addEventListener('load', (e) => {
            const { open } = this.props;

            if (open) {
                let data = null;
                let success = null;
                try {
                    data = JSON.parse(e.target.result);
                    success = true;
                } catch (ex) {
                    success = false;
                }

                this.setState((prevState) => {
                    return {
                        ...prevState,
                        pending: false,
                        data,
                        fileSuccessfullyParsed: success,
                    };
                });
            }
        });

        reader.readAsText(files[0]);

        this.setState((prevState) => {
            return {
                ...prevState,
                pending: true,
                data: null,
                filename: files[0].name,
            };
        });
    }

    submitClicked() {
        // TODO submit data action
        console.log('Submit Clicked');

        this.hideModal();
    }

    render() {

        const { 
            open, 
            totalWidth 
        } = this.props;

        const { 
            filename, 
            fileSuccessfullyParsed, 
            pending, 
            data 
        } = this.state;

        const width = totalWidth / 2;

        let metaData = null;
        if (pending) {
            metaData = (<Spinner label="Uploading" />);
        } else if (data !== null) {
            if (fileSuccessfullyParsed) {
                metaData = (<p>META DATA : Work in progress</p>);
            } else {
                metaData = (
                    <Stack.Item align="center">
                        <p>Failed to parse file</p>
                    </Stack.Item>
                );
            }
        } else {
            metaData = (
                <Stack.Item align="center">
                    <p>Please upload file</p>
                </Stack.Item>
            );
        }

        return (
            <ModalWrapper
                open={open}
                title="Upload"
                hideModal={this.hideModal}
                width={width}
            >
                <input
                    type="file"
                    id="fileupload"
                    style={{ display: 'none' }}
                    onChange={this.fileChanged}
                    accept=".json, application/json, application/JSON"
                />

                <Stack>
                    <Stack horizontal tokens={{ childrenGap: '2' }}>

                        <Stack.Item>
                            <PrimaryButton
                                text="File"
                                iconProps={{ iconName: 'Upload' }}
                                allowDisabledFocus
                                onClick={this.uploadClicked}
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
                                'submit': this.submitClicked, 
                                'cancel': this.hideModal
                            }}
                            isPrimary={{
                                'submit': true, 
                                'cancel': false
                            }}
                        />
                    </Stack.Item>
                </Stack>
            </ModalWrapper>
        );
    }
}