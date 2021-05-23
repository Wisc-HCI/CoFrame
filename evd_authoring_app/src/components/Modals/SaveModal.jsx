import React, { useContext } from 'react';

import { Stack } from '@fluentui/react/lib/Stack';
import { TextField } from '@fluentui/react/lib/TextField';

import { ModalWrapper } from './ModalWrapper';
import { ModalControlButtons } from './ModalControlButtons';

import useApplicationStore from '../../stores/ApplicationStore';
import useGuiStore from '../../stores/GuiStore';


export const SaveModal = (props) => {

    const {save, filename, setFilename} = useApplicationStore(state=>({
        save:state.save,
        filename:state.filename,
        setFilename:state.setFilename,
    }));
    const {closeModal} = useGuiStore(state=>({
        setActiveModal:state.setActiveModal,
        closeModal:state.closeModal
    }))

    return (
        <ModalWrapper
            name="saveas"
            title="Save As"
            closeCb={closeModal}
        >
            <Stack>
                <br />

                <Stack.Item align="center">
                    <TextField 
                        id="saveas-project-name"
                        defaultValue={filename} 
                    />
                </Stack.Item>

                <br />

                <Stack.Item align="center">

                    <ModalControlButtons 
                        order={['submit','cancel']} 
                        callbacks={{
                            'submit': () => {
                                setFilename(document.getElementById('saveas-project-name').value);
                                save();
                                closeModal(); 
                            }, 
                            'cancel': closeModal
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