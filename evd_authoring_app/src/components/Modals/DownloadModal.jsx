import React, { useState, useContext }  from 'react';

import { saveAs } from 'file-saver';

import { Stack } from '@fluentui/react/lib/Stack';

import { ModalWrapper } from './ModalWrapper';
import { ModalControlButtons } from './ModalControlButtons';

import useApplicationStore from '../../stores/ApplicationStore';
import useGuiStore from '../../stores/GuiStore';
import useEvdStore from '../../stores/EvdStore';

import { ApplicationContext, EvDScriptContext, ModalContext } from '../../contexts';


export const DownloadModal = (props) => {

    const [downloading, setDownloading] = useState(false);
    
    const {activeModal, closeModal, filename} = useApplicationStore(state=>(
        {
            activeModal:state.activeModal,
            closeModal:state.closeModal,
            filename:state.filename
        }));
    const evdProgram = useEvdStore(state=>state.program);


    if (activeModal === 'download' && !downloading) {

        const program = evdProgram;
        const text = JSON.stringify((program === null) ? {} : program.toDict());

        const blob = new Blob([text], { type: 'text/plain;charset=utf-8' });
        saveAs(blob, `${filename}.json`);
        setDownloading(true);
    }

    const closeFnt = () => {
        closeModal();
        setDownloading(false);
    };

    return (
        <ModalWrapper
            name="download"
            title="Download"closeCb={closeFnt}
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
                        callbacks={{'close': closeFnt}}
                        isPrimary={{'close': true}}
                    />
                </Stack.Item>
            </Stack>
        </ModalWrapper>
    );
}