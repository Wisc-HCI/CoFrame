import React  from 'react';

import {
    TextField,
    PrimaryButton,
    Stack,
    Spinner,
} from 'office-ui-fabric-react';

import { MetaData } from './MetaData';
import { ModalWrapper } from '../ModalWrapper';
import { ModalControlButtons } from '../ModalControlButtons';

import { ApplicationContext } from '../../../contexts';


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

    submitClicked(service) {
        service.loadFromFile(this.state.data, this.state.filename.split('.').slice(0, -1).join('.'))
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
                        <ApplicationContext.Consumer>
                            { appValue => (
                                <ModalControlButtons 
                                    order={['submit','cancel']} 
                                    callbacks={{
                                        'submit': () => {this.submitClicked(appValue.service)}, 
                                        'cancel': this.hideModal
                                    }}
                                    isPrimary={{
                                        'submit': true, 
                                        'cancel': false
                                    }}
                                    disabled={{'submit': !fileSuccessfullyParsed}}
                                />
                            )}
                        </ApplicationContext.Consumer> 
                        
                    </Stack.Item>
                </Stack>
            </ModalWrapper>
        );
    }
}