import React  from 'react';

import { saveAs } from 'file-saver';

import { Stack } from 'office-ui-fabric-react';

import { ModalWrapper } from './ModalWrapper';
import { ModalControlButtons } from './ModalControlButtons';


export class DownloadModal extends React.Component {

    constructor(props) {
        super(props);

        this.state = {
            downloading: false,
            text: '{}',
            filename: 'untitled.json',
        };

        this.hideModal = this.hideModal.bind(this);
    }

    hideModal() {
        this.setState((prevState) => {
            return {
                ...prevState,
                downloading: false,
            };
        });

        const { closeModal } = this.props;
        closeModal('download');
    }

    render() {

        const { 
            open, 
            totalWidth 
        } = this.props;
        
        const { 
            downloading, 
            text, 
            filename 
        } = this.state;

        const width = totalWidth / 2;

        if (open && !downloading) {
            const blob = new Blob([text], { type: 'text/plain;charset=utf-8' });
            saveAs(blob, filename);

            this.setState((prevState) => {
                return {
                    ...prevState,
                    downloading: true,
                };
            });
        }

        return (
            <ModalWrapper
                open={open}
                title="Download"
                hideModal={this.hideModal}
                width={width}
            >
                <Stack>
                    <br />

                    <Stack.Item align="center">
                        <p>File is downloading. Press the <i><b>Close</b></i> button to return to design.</p>
                    </Stack.Item>

                    <br />

                    <Stack.Item align="center">
                        <ModalControlButtons 
                            order={['close']} 
                            callbacks={{'close': this.hideModal}}
                            isPrimary={{'close': true}}
                        />
                    </Stack.Item>
                </Stack>
            </ModalWrapper>
        );
    }
}