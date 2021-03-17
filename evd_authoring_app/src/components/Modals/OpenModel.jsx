import React from 'react';

import { Stack } from 'office-ui-fabric-react';

import { ModalWrapper } from './ModalWrapper';
import { ModalControlButtons } from './ModalControlButtons';


export class OpenModal extends React.Component {
    
    constructor(props) {
        super(props);

        this.state = {
            selectedOption: null,
        }

        this.submitClicked = this.submitClicked.bind(this);
    }

    submitClicked() {
        // TODO submit selected file action

        this.props.closeModal('open');
    }

    render() {
        const { 
            open, 
            totalWidth,
            closeModal
        } = this.props;

        const { selectedOption } = this.state;

        const width = totalWidth / 2;

        let visibleBtns = [];
        if (selectedOption !== null) {
            visibleBtns.push('submit');
        }
        visibleBtns.push('cancel');

        return (
            <ModalWrapper
                open={open}
                title="Open"
                hideModal={() => {closeModal('open')}}
                width={width}
            >
                <Stack>
                    <br />

                    <Stack.Item align="center">
                        <p>Open options menu coming soon!</p>
                    </Stack.Item>

                    <br />

                    <Stack.Item align="center">

                        <ModalControlButtons 
                            order={visibleBtns} 
                            callbacks={{
                                'submit': this.submitClicked, 
                                'cancel': () => {closeModal('open')}
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