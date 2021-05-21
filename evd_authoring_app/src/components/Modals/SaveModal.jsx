import React, { useContext } from 'react';

import { Stack } from '@fluentui/react/lib/Stack';
import { TextField } from '@fluentui/react/lib/TextField';

import { ModalWrapper } from './ModalWrapper';
import { ModalControlButtons } from './ModalControlButtons';

import { 
    ModalContext, 
    ApplicationContext 
} from '../../contexts';


export const SaveModal = (props) => {

    const { totalWidth } = props;

    const modalContext = useContext(ModalContext);
    const appContext = useContext(ApplicationContext);

    const saveFnt = () => {
        appContext.service.saveToFile();
    };

    const closeFnt = () => {
        modalContext.closeModal('saveas');
        modalContext.closeModal('save');
    };

    if (modalContext.state['save']) {
        if (!appContext.filenameHasChanged && !modalContext.state['saveas']) {
            modalContext.openModal('saveas');
        } else {
            saveFnt();
            modalContext.closeModal('save');
        }
    }

    return (
        <ModalWrapper
            name="saveas"
            title="Save As"
            totalWidth={totalWidth}
            closeCb={closeFnt}
        >
            <Stack>
                <br />

                <Stack.Item align="center">
                    <TextField 
                        id="saveas-project-name"
                        defaultValue={appContext.filename} 
                    />
                </Stack.Item>

                <br />

                <Stack.Item align="center">

                    <ModalControlButtons 
                        order={['submit','cancel']} 
                        callbacks={{
                            'submit': () => { 
                                appContext.service.filename = document.getElementById('saveas-project-name').value;
                                saveFnt();
                                closeFnt(); 
                            }, 
                            'cancel': closeFnt
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
};