import React, { useState, useContext } from 'react';

import { Stack } from '@fluentui/react/lib/Stack';

import { ModalWrapper } from './ModalWrapper';
import { ModalControlButtons } from './ModalControlButtons';

import { ModalContext } from '../../contexts';


export const OpenModal = (props) => {

    const { totalWidth } = props;

    const [selectedOption, setSelectedOption] = useState(null);

    const modalContext = useContext(ModalContext);

    let visibleBtns = [];
    if (selectedOption !== null) {
        visibleBtns.push('submit');
    }
    visibleBtns.push('cancel');

    return (
        <ModalWrapper
            name="open"
            title="Open"
            totalWidth={totalWidth}
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
                            'submit': () => { modalContext.closeModal('open') }, 
                            'cancel': () => { modalContext.closeModal('open') }
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